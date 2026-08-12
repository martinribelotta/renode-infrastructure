//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
// Models just enough of the STM32H7 OCTOSPI peripheral in Hyperbus mode to satisfy this
// project's HyperRam driver (Platform/Src/HyperRam.cpp): register-space read/write of an
// IS66/67WVH64M8DALL/BLL's ID0/ID1/CR0/CR1 words (dual-die, die select = register-space
// word address bit 24 - see HyperRam.hpp's dieWordBase()), and accepting the switch to
// memory-mapped mode. One instance per OCTOSPI controller (OCTOSPI1 @ 0x52005000,
// OCTOSPI2 @ 0x5200A000 register base - see this board's .repl.in) - both dies of both
// HyperRAM chips are modeled identically since nothing in this driver depends on them
// differing.
//
// The memory-mapped window itself (OCTOSPI1_BASE 0x90000000 / OCTOSPI2_BASE 0x70000000,
// both 64MB) is a separate Memory.MappedMemory region in the platform description,
// always live regardless of FMODE - real hardware only allows AXI access there once
// switched to memory-mapped mode, but nothing in this project depends on catching an
// out-of-sequence access, so that simplification is harmless.
//
// Everything about the actual Hyperbus wire protocol - RWDS/DQS timing, latency cycles,
// linear vs wrapped burst addressing during memory-mapped traffic - is not modeled; only
// the register-space transaction sequence HAL_OSPI_HyperbusCmd + HAL_OSPI_Receive/
// Transmit produce (stm32h7xx_hal_ospi.c), which HyperRam::init() uses for ID
// verification and CR0 programming before switching to memory-mapped mode. Registers
// this project's driver never touches (DCR1-4, CCR, WCCR, TCR, IR, ABR, HLCR, ...) are
// deliberately left undefined - reads/writes there just get the standard "unhandled
// register" log message, same as every other peripheral in this platform that only
// models what the firmware actually exercises.
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.SPI
{
    public sealed class STM32_OCTOSPI_HyperRAM : IDoubleWordPeripheral, IBytePeripheral, IKnownSize
    {
        public STM32_OCTOSPI_HyperRAM(IMachine machine)
        {
            registers = new DoubleWordRegisterCollection(this);
            DefineRegisters();
        }

        public void Reset()
        {
            registers.Reset();
            readBuffer.Clear();
            writeBuffer.Clear();
            Array.Clear(cr0, 0, cr0.Length);
            Array.Clear(cr1, 0, cr1.Length);
            lastAddress = 0;
        }

        public uint ReadDoubleWord(long offset) => registers.Read(offset);

        public void WriteDoubleWord(long offset, uint value) => registers.Write(offset, value);

        public byte ReadByte(long offset)
        {
            if(offset != (long)Registers.Data)
            {
                this.Log(LogLevel.Warning, "Byte access only supported on the Data register, ignoring read at offset 0x{0:X}", offset);
                return 0;
            }
            if(!readBuffer.TryDequeue(out var result))
            {
                this.Log(LogLevel.Warning, "Data register read with nothing pending, returning 0");
                return 0;
            }
            return result;
        }

        public void WriteByte(long offset, byte value)
        {
            if(offset != (long)Registers.Data)
            {
                this.Log(LogLevel.Warning, "Byte access only supported on the Data register, ignoring write at offset 0x{0:X}", offset);
                return;
            }
            if(functionalMode.Value != ModeOfOperation.IndirectWrite)
            {
                this.Log(LogLevel.Warning, "Data register written outside IndirectWrite mode (mode: {0}), ignoring", functionalMode.Value);
                return;
            }
            writeBuffer.Enqueue(value);
            if(writeBuffer.Count >= RegisterTransferBytes)
            {
                CommitRegisterWrite();
            }
        }

        public long Size => 0x400;

        // Never actually asserted - this project's driver only uses blocking/polling HAL
        // calls, never OSPI interrupts - but the platform description wires it to the
        // NVIC anyway (matching real hardware) since Renode's `->` connector syntax
        // requires a GPIO property to exist on the target.
        public GPIO IRQ { get; } = new GPIO();

        private void DefineRegisters()
        {
            Registers.Control.Define(registers)
                .WithFlag(0, name: "EN")
                .WithReservedBits(1, 27)
                .WithEnumField(28, 2, out functionalMode, name: "FMODE")
                .WithReservedBits(30, 2);

            Registers.Status.Define(registers)
                .WithFlag(0, FieldMode.Read, name: "TEF")
                .WithFlag(1, FieldMode.Read, valueProviderCallback: _ => true, name: "TCF")
                .WithFlag(2, FieldMode.Read, valueProviderCallback: _ => true, name: "FTF")
                .WithFlag(3, FieldMode.Read, name: "SMF")
                .WithFlag(4, FieldMode.Read, name: "TOF")
                .WithFlag(5, FieldMode.Read, name: "BUSY")
                .WithReservedBits(6, 26);

            Registers.Address.Define(registers)
                .WithValueField(0, 32, name: "AR", writeCallback: (_, value) => OnAddressRegisterWritten(value));

            // Unlike CCR/WCCR/DCR1 (write-and-forget, safe to leave undefined), HAL_OSPI_Receive
            // reads DLR back (READ_REG(DLR) + 1) to compute how many bytes to pull from DR - if
            // this stayed undefined, that readback would return 0 regardless of what was written,
            // truncating every 2-byte register read to 1 byte and corrupting the low byte of every
            // ID/CR value (HyperRam::readRegister() zero-initializes its buffer, so the effect is
            // silently wrong data rather than a crash - only surfaces as failed ID verification).
            Registers.DataLength.Define(registers)
                .WithValueField(0, 32, name: "DLR");
        }

        // Only fires the register-space read on the *second* AR write of a read sequence:
        // HAL_OSPI_HyperbusCmd writes AR once while FMODE is still 0/IndirectWrite (its
        // own reset value); HAL_OSPI_Receive is what re-writes AR, after first setting
        // FMODE=IndirectRead. That second address write is what the real H7 OSPI
        // controller triggers a Hyperbus register read from, so this mirrors that rather
        // than triggering on every AR write.
        private void OnAddressRegisterWritten(ulong value)
        {
            lastAddress = value;
            if(functionalMode.Value != ModeOfOperation.IndirectRead)
            {
                return;
            }
            var found = TryReadRegisterSpace(value, out var result);
            readBuffer.Clear();
            if(!found)
            {
                this.Log(LogLevel.Warning, "Register-space read at unrecognized byte address 0x{0:X}, returning 0", value);
                result = 0;
            }
            // Table 3.4: register space is Big-endian - the byte transferred first on the
            // bus is Word[15:8], matching HyperRam::readRegister()'s `raw[0]<<8 | raw[1]`.
            readBuffer.Enqueue((byte)(result >> 8));
            readBuffer.Enqueue((byte)(result & 0xFF));
        }

        private void CommitRegisterWrite()
        {
            // writeBuffer now has exactly RegisterTransferBytes bytes, MSB first - mirrors
            // HyperRam::writeRegister()'s `raw[0] = value>>8, raw[1] = value&0xFF`.
            var high = writeBuffer.Dequeue();
            var low = writeBuffer.Dequeue();
            var value = (ushort)((high << 8) | low);
            var written = TryWriteRegisterSpace(lastAddress, value);
            if(!written)
            {
                this.Log(LogLevel.Warning, "Register-space write at unrecognized byte address 0x{0:X} (value 0x{1:X4}), ignoring", lastAddress, value);
            }
        }

        private bool TryReadRegisterSpace(ulong byteAddress, out ushort value)
        {
            if(!TryDecodeAddress(byteAddress, out var die, out var localWordOffset))
            {
                value = 0;
                return false;
            }
            switch(localWordOffset)
            {
            case Id0WordOffset:
                value = Id0Value;
                return true;
            case Id1WordOffset:
                value = Id1Value;
                return true;
            case Cr0WordOffset:
                value = cr0[die];
                return true;
            case Cr1WordOffset:
                value = cr1[die];
                return true;
            default:
                value = 0;
                return false;
            }
        }

        private bool TryWriteRegisterSpace(ulong byteAddress, ushort value)
        {
            if(!TryDecodeAddress(byteAddress, out var die, out var localWordOffset))
            {
                return false;
            }
            switch(localWordOffset)
            {
            case Cr0WordOffset:
                cr0[die] = value;
                return true;
            case Cr1WordOffset:
                cr1[die] = value;
                return true;
            default:
                return false;
            }
        }

        // HyperRamMemory::WordsPerDie (HyperRamMemory.hpp): each 256Mb die covers 2^24
        // word addresses, so that bit is exactly the die-select bit (also documented on
        // HyperRam.hpp's dieWordBase()).
        private static bool TryDecodeAddress(ulong byteAddress, out int die, out ulong localWordOffset)
        {
            var wordIndex = byteAddress / 2;
            die = (int)(wordIndex / WordsPerDie);
            localWordOffset = wordIndex % WordsPerDie;
            if(die > 1)
            {
                die = 0;
                return false;
            }
            return true;
        }

        private IEnumRegisterField<ModeOfOperation> functionalMode;
        private ulong lastAddress;

        private readonly ushort[] cr0 = new ushort[2];
        private readonly ushort[] cr1 = new ushort[2];
        private readonly Queue<byte> readBuffer = new Queue<byte>();
        private readonly Queue<byte> writeBuffer = new Queue<byte>();

        private readonly DoubleWordRegisterCollection registers;

        private const int RegisterTransferBytes = 2;
        private const ulong WordsPerDie = 1UL << 24;

        private const ulong Id0WordOffset = 0x0000000;
        private const ulong Id1WordOffset = 0x0000001;
        private const ulong Cr0WordOffset = 0x0000800;
        private const ulong Cr1WordOffset = 0x0000801;

        // ID0[3:0]=0b0011 (ISSI manufacturer), ID1[3:0]=0b0001 (HyperRAM device type) -
        // the only bits HyperRam::verifyId() actually checks (Table 5.2/5.3); other bits
        // are left zeroed since nothing in this driver reads them.
        private const ushort Id0Value = 0b0011;
        private const ushort Id1Value = 0b0001;

        private enum ModeOfOperation
        {
            IndirectWrite = 0b00,
            IndirectRead = 0b01,
            AutomaticStatusPolling = 0b10,
            MemoryMapped = 0b11,
        }

        private enum Registers
        {
            Control = 0x00,
            Status = 0x20,
            DataLength = 0x40,
            Address = 0x48,
            Data = 0x50,
        }
    }
}
