//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
// Functional model of one block (A or B) of an STM32 SAI (Serial Audio
// Interface) peripheral, written for the "Opinionated Chaos" audio pedal
// project (STM32H723ZG + CS4272 codec over SAI1, I2S standard, 24-bit,
// stereo, 192 kHz).
//
// This does NOT model real FIFO backpressure, MCLK/SCK/FS clock generation,
// or real-time audio pacing - the driving HAL (stm32h7xx_hal_sai.c) only
// needs SR.FLVL to never read as exactly "empty" or "full" and SR.FREQ to
// track SAIEN for its FillFifo()/interrupt-driven pump loops to complete
// without hanging. Given that, transfers are effectively instantaneous:
// every DR write/read is immediately forwarded to/pulled from an audio
// sink/source, which is what lets this stay usable in Renode's non-KVM,
// software-emulated CPU without needing to simulate ~192,000 FIFO
// interrupts of virtual time per second.
//
// Content fidelity (not timing fidelity) is the goal: DR is a 32-bit
// container and this model does not assume where in it the significant
// bits live (that is an application-level convention, not enforced by the
// HAL at this layer) - it stores/loads the raw 32-bit word as one channel
// sample of a standard 32-bit-PCM WAV file, interleaved by channel count,
// so captures are directly listenable/inspectable and injected files don't
// need any STM32-specific pre-processing beyond bit depth.
using System;
using System.Collections.Generic;
using System.IO;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sound
{
    public class STM32_SAI : IDoubleWordPeripheral, IKnownSize, IDisposable
    {
        public STM32_SAI(IMachine machine, string audioInputFile = null, string audioOutputFile = null,
            uint sampleRate = 192000, uint channels = 2)
        {
            IRQ = new GPIO();

            if(audioInputFile != null)
            {
                reader = new WavPcm32Reader(audioInputFile);
            }
            if(audioOutputFile != null)
            {
                writer = new WavPcm32Writer(audioOutputFile, sampleRate, channels);
            }

            // Drives DmaRequest below. This does NOT run at the real 192kHz Fs (see the class
            // header comment on "content fidelity, not timing fidelity") - nothing on the
            // firmware side measures wall-clock audio timing, HAL_SAI_Transmit_DMA()/Receive_DMA()
            // just need their half/complete DMA callbacks to eventually fire so
            // Platform/Src/Audio.cpp's play/record task loop makes progress instead of hanging
            // forever on a semaphore. DmaRequestRate (see below, a few kHz) is cheap to simulate on
            // a non-accelerated software CPU while still moving a meaningful number of samples over
            // the ~15s virtual boot smoke test.
            dmaRequestTimer = new LimitTimer(
                  machine.ClockSource, 1000000, this, "dmaRequestClock",
                  limit: 1000000 / DmaRequestRate,
                  eventEnabled: true,
                  direction: Direction.Ascending,
                  enabled: false,
                  autoUpdate: false,
                  workMode: WorkMode.Periodic);
            dmaRequestTimer.LimitReached += OnDmaRequestTimerTick;

            var registerMap = new Dictionary<long, DoubleWordRegister>
            {
                {(long)Registers.Configuration1, new DoubleWordRegister(this)
                    .WithValueField(0, 2, out mode, name: "MODE")
                    .WithValueField(2, 2, name: "PRTCFG")
                    .WithReservedBits(4, 1)
                    .WithValueField(5, 3, name: "DS")
                    .WithFlag(8, name: "LSBFIRST")
                    .WithFlag(9, name: "CKSTR")
                    .WithValueField(10, 2, name: "SYNCEN")
                    .WithFlag(12, name: "MONO")
                    .WithFlag(13, name: "OUTDRIV")
                    .WithReservedBits(14, 2)
                    .WithFlag(16, out saiEnabled, name: "SAIEN", changeCallback: (_, __) => { UpdateInterrupts(); UpdateDmaRequestTimer(); })
                    .WithFlag(17, out dmaEnabled, name: "DMAEN", changeCallback: (_, __) => UpdateDmaRequestTimer())
                    .WithReservedBits(18, 1)
                    .WithFlag(19, name: "NODIV")
                    .WithValueField(20, 6, name: "MCKDIV")
                    .WithReservedBits(26, 6)
                },
                {(long)Registers.Configuration2, new DoubleWordRegister(this)
                    .WithValueField(0, 3, name: "FTH")
                    .WithFlag(3, FieldMode.Write, name: "FFLUSH") // Nothing to flush: transfers are never actually buffered here.
                    .WithFlag(4, name: "TRIS")
                    .WithFlag(5, name: "MUTE")
                    .WithFlag(6, name: "MUTEVAL")
                    .WithValueField(7, 6, name: "MUTECNT")
                    .WithReservedBits(13, 1)
                    .WithValueField(14, 2, name: "COMP")
                    .WithReservedBits(16, 16)
                },
                {(long)Registers.FrameConfiguration, new DoubleWordRegister(this)
                    .WithTag("FRL", 0, 8)
                    .WithTag("FSALL", 8, 7)
                    .WithReservedBits(15, 1)
                    .WithFlag(16, name: "FSDEF")
                    .WithFlag(17, name: "FSPOL")
                    .WithFlag(18, name: "FSOFF")
                    .WithReservedBits(19, 13)
                },
                {(long)Registers.SlotRegister, new DoubleWordRegister(this)
                    .WithTag("FBOFF", 0, 5)
                    .WithReservedBits(5, 1)
                    .WithValueField(6, 2, name: "SLOTSZ")
                    .WithValueField(8, 4, name: "NBSLOT")
                    .WithReservedBits(12, 4)
                    .WithValueField(16, 16, name: "SLOTEN")
                },
                {(long)Registers.InterruptMask, new DoubleWordRegister(this)
                    .WithFlag(0, name: "OVRUDRIE")
                    .WithFlag(1, name: "MUTEDETIE")
                    .WithFlag(2, name: "WCKCFGIE")
                    .WithFlag(3, out freqInterruptEnabled, name: "FREQIE", changeCallback: (_, __) => UpdateInterrupts())
                    .WithFlag(4, name: "CNRDYIE")
                    .WithFlag(5, name: "AFSDETIE")
                    .WithFlag(6, name: "LFSDETIE")
                    .WithReservedBits(7, 25)
                },
                {(long)Registers.Status, new DoubleWordRegister(this)
                    .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => false, name: "OVRUDR")
                    .WithFlag(1, FieldMode.Read, valueProviderCallback: _ => false, name: "MUTEDET")
                    .WithFlag(2, FieldMode.Read, valueProviderCallback: _ => false, name: "WCKCFG")
                    .WithFlag(3, FieldMode.Read, valueProviderCallback: _ => saiEnabled.Value, name: "FREQ")
                    .WithFlag(4, FieldMode.Read, valueProviderCallback: _ => false, name: "CNRDY")
                    .WithFlag(5, FieldMode.Read, valueProviderCallback: _ => false, name: "AFSDET")
                    .WithFlag(6, FieldMode.Read, valueProviderCallback: _ => false, name: "LFSDET")
                    .WithReservedBits(7, 9)
                    // Reported as a constant mid-level fill so the HAL's "not empty"/"not full" spin-waits
                    // (SAI_FillFifo, the blocking Transmit/Receive paths) never block: every DR access is
                    // forwarded immediately, so there is no real occupancy to report.
                    .WithValueField(16, 3, FieldMode.Read, valueProviderCallback: _ => saiEnabled.Value ? 2u : 0u, name: "FLVL")
                    .WithReservedBits(19, 13)
                },
                {(long)Registers.ClearFlag, new DoubleWordRegister(this)
                    .WithFlag(0, FieldMode.Write, name: "COVRUDR")
                    .WithFlag(1, FieldMode.Write, name: "CMUTEDET")
                    .WithFlag(2, FieldMode.Write, name: "CWCKCFG")
                    .WithFlag(3, FieldMode.Write, name: "CFREQ")
                    .WithFlag(4, FieldMode.Write, name: "CCNRDY")
                    .WithFlag(5, FieldMode.Write, name: "CAFSDET")
                    .WithFlag(6, FieldMode.Write, name: "CLFSDET")
                    .WithReservedBits(7, 25)
                },
                {(long)Registers.Data, new DoubleWordRegister(this)
                    .WithValueField(0, 32, valueProviderCallback: _ => ReadSample(), writeCallback: (_, value) => WriteSample((uint)value), name: "DATA")
                },
            };
            registers = new DoubleWordRegisterCollection(this, registerMap);
        }

        public void Reset()
        {
            registers.Reset();
            IRQ.Set(false);
            dmaRequestTimer.Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            return registers.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            registers.Write(offset, value);
        }

        public void Dispose()
        {
            writer?.Dispose();
            reader?.Dispose();
        }

        // Exactly one block's register window (CR1..DR, offsets 0x00-0x1C); must not overlap the
        // other block, which is registered 0x20 bytes further up the bus (see the platform .repl).
        public long Size => 0x20;

        public GPIO IRQ { get; }

        // Pulsed periodically (see dmaRequestTimer, constructor) while SAIEN and DMAEN are both
        // set, so a DMA controller wired to it (dmamux1, see the platform .repl) keeps performing
        // peripheral<->memory transfers for as long as the firmware's circular-mode DMA stream is
        // running - unlike STM32_ADC's DMARequest (one pulse per one-shot conversion), this one
        // free-runs because SAI1 in this firmware is always started in circular DMA mode.
        public GPIO DmaRequest { get; } = new GPIO();

        // MODE[0] distinguishes transmitter (0: master/slave TX) from receiver (1: master/slave RX),
        // matching SAI_xCR1_MODE encoding (00 Master Tx, 01 Master Rx, 10 Slave Tx, 11 Slave Rx).
        private bool IsReceiver => (mode.Value & 0x1) != 0;

        private uint ReadSample()
        {
            if(!IsReceiver)
            {
                // Real hardware doesn't define a meaningful read of a TX-direction DR; give back the
                // last value transmitted rather than a hardcoded constant, closest available approximation.
                return lastTransmitted;
            }
            var sample = reader?.NextSample() ?? 0u;
            this.Log(LogLevel.Noisy, "RX sample 0x{0:X8}", sample);
            return sample;
        }

        private void WriteSample(uint value)
        {
            if(IsReceiver)
            {
                this.Log(LogLevel.Warning, "Write to DR while configured as a receiver (MODE={0}); ignoring", mode.Value);
                return;
            }
            lastTransmitted = value;
            this.Log(LogLevel.Noisy, "TX sample 0x{0:X8}", value);
            writer?.WriteSample(value);
        }

        private void UpdateInterrupts()
        {
            IRQ.Set(saiEnabled.Value && freqInterruptEnabled.Value);
        }

        private void UpdateDmaRequestTimer()
        {
            dmaRequestTimer.Enabled = saiEnabled.Value && dmaEnabled.Value;
        }

        private void OnDmaRequestTimerTick()
        {
            // Issue DMA peripheral request, which when mapped to a DMA controller stream
            // (dmamux1, see the platform .repl) will trigger a peripheral<->memory transfer.
            // WorkMode.Periodic re-arms this on its own (no manual re-enable needed here,
            // unlike STM32_ADC's OneShot mode) - it keeps firing until UpdateDmaRequestTimer()
            // stops it, i.e. until SAIEN or DMAEN goes low.
            DmaRequest.Set();
            DmaRequest.Unset();
        }

        private readonly DoubleWordRegisterCollection registers;
        private readonly WavPcm32Reader reader;
        private readonly WavPcm32Writer writer;
        private readonly LimitTimer dmaRequestTimer;
        private readonly IValueRegisterField mode;
        private readonly IFlagRegisterField saiEnabled;
        private readonly IFlagRegisterField dmaEnabled;
        private readonly IFlagRegisterField freqInterruptEnabled;
        private uint lastTransmitted;

        // Virtual DMA-request rate (Hz), deliberately far below the real 192kHz Fs - see the
        // constructor's comment on dmaRequestTimer for why. 4kHz keeps a ~15s virtual-time boot
        // smoke test (sim/run.sh) cheap to simulate while still moving tens of thousands of
        // samples, enough for the firmware's DMA half/complete callbacks to fire repeatedly.
        private const uint DmaRequestRate = 4000;

        private enum Registers
        {
            Configuration1 = 0x0,
            Configuration2 = 0x4,
            FrameConfiguration = 0x8,
            SlotRegister = 0xC,
            InterruptMask = 0x10,
            Status = 0x14,
            ClearFlag = 0x18,
            Data = 0x1C,
        }
    }
}
