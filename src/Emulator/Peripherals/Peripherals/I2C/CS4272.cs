//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
// Cirrus Logic CS4272 audio codec, control port only (I2C mode) - written for the
// "Opinionated Chaos" audio pedal project. The codec's actual audio path (DAC/ADC over
// I2S/SAI) isn't this chip's job to simulate; see Sound.STM32_SAI for that. This just
// needs to be a well-behaved I2C target so the firmware's codec init/config code has
// something real to talk to and read back from.
//
// Register access (CS4272 datasheet section 6.2, "I2C Mode"): the first byte of a write
// is not register data, it's the MAP (Memory Address Pointer) - bit 7 (INCR) enables
// auto-increment across subsequent bytes, bits 3:0 select the register (01h-08h; 00h is
// not a valid register). A read continues from whatever MAP a prior write last set,
// incrementing under the same INCR rule - this is the same repeated-START register-
// pointer idiom as most I2C EEPROMs/sensors, just with an extra auto-increment enable
// bit riding along in the pointer byte instead of it being implicit.
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class CS4272 : II2CPeripheral, IProvidesRegisterCollection<ByteRegisterCollection>
    {
        public CS4272()
        {
            RegistersCollection = new ByteRegisterCollection(this);
            DefineRegisters();
            Reset();
        }

        public void Write(byte[] data)
        {
            foreach(var b in data)
            {
                WriteByte(b);
            }
        }

        public byte[] Read(int count)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = RegistersCollection.Read(address);
                Advance();
            }
            return result;
        }

        public void FinishTransmission()
        {
            // The MAP is only ever set by a write, and a read continues from wherever it was
            // left - i.e. it survives a STOP/repeated-START on purpose. Nothing to do here.
            haveMap = false;
        }

        public void Reset()
        {
            RegistersCollection.Reset();
            address = 0;
            incrementEnabled = false;
            haveMap = false;
        }

        public ByteRegisterCollection RegistersCollection { get; }

        private void WriteByte(byte b)
        {
            if(!haveMap)
            {
                incrementEnabled = (b & 0x80) != 0;
                address = (byte)(b & 0x0F);
                haveMap = true;
                this.Log(LogLevel.Noisy, "MAP set to register 0x{0:X2} (INCR={1})", address, incrementEnabled);
                return;
            }

            this.Log(LogLevel.Noisy, "Write 0x{0:X2} to register 0x{1:X2}", b, address);
            RegistersCollection.Write(address, b);
            Advance();
        }

        private void Advance()
        {
            if(incrementEnabled)
            {
                address++;
            }
        }

        private void DefineRegisters()
        {
            Registers.ModeControl1.Define(RegistersCollection)
                .WithValueField(0, 3, name: "DAC_DIF")
                .WithFlag(3, name: "MS")
                .WithValueField(4, 2, name: "RATIO")
                .WithValueField(6, 2, name: "FM");

            Registers.DACControl.Define(RegistersCollection, 0x80)
                .WithFlag(0, name: "INV_B")
                .WithFlag(1, name: "INV_A")
                .WithFlag(2, name: "RMP_DN")
                .WithFlag(3, name: "RMP_UP")
                .WithValueField(4, 2, name: "DEM")
                .WithFlag(6, name: "FILT_SEL")
                .WithFlag(7, name: "AMUTE");

            Registers.DACVolumeAndMixingControl.Define(RegistersCollection, 0x29)
                .WithValueField(0, 4, name: "ATAPI")
                .WithFlag(4, name: "ZEROCROSS")
                .WithFlag(5, name: "SOFT")
                .WithFlag(6, name: "B_EQ_A")
                .WithReservedBits(7, 1);

            Registers.DACChannelAVolumeControl.Define(RegistersCollection)
                .WithValueField(0, 7, name: "VOL")
                .WithFlag(7, name: "MUTE");

            Registers.DACChannelBVolumeControl.Define(RegistersCollection)
                .WithValueField(0, 7, name: "VOL")
                .WithFlag(7, name: "MUTE");

            Registers.ADCControl.Define(RegistersCollection)
                .WithFlag(0, name: "HPF_DISABLE_B")
                .WithFlag(1, name: "HPF_DISABLE_A")
                .WithFlag(2, name: "MUTEB")
                .WithFlag(3, name: "MUTEA")
                .WithFlag(4, name: "ADC_DIF")
                .WithFlag(5, name: "DITHER16")
                .WithReservedBits(6, 2);

            Registers.ModeControl2.Define(RegistersCollection)
                .WithFlag(0, name: "PDN")
                .WithFlag(1, name: "CPEN")
                .WithFlag(2, name: "FREEZE")
                .WithFlag(3, name: "MUTECA_EQ_B")
                .WithFlag(4, name: "LOOP")
                .WithReservedBits(5, 3);

            // Read-only; PART(3:0)=0h and REV(3:0)=0h for both silicon revisions per the datasheet.
            Registers.ChipID.Define(RegistersCollection)
                .WithValueField(0, 8, FieldMode.Read, valueProviderCallback: _ => 0, name: "REV_PART");
        }

        private byte address;
        private bool incrementEnabled;
        private bool haveMap;

        private enum Registers
        {
            ModeControl1 = 0x01,
            DACControl = 0x02,
            DACVolumeAndMixingControl = 0x03,
            DACChannelAVolumeControl = 0x04,
            DACChannelBVolumeControl = 0x05,
            ADCControl = 0x06,
            ModeControl2 = 0x07,
            ChipID = 0x08,
        }
    }
}
