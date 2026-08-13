//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.DMA;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class STM32H7_ADC : STM32_ADC_Common
    {
        // resolutionRange defaults to Bits8_16, matching ADC1/ADC2 on the H72x/H73x
        // sub-family (the only instances with a 16-bit-capable RES field per the HAL
        // header, stm32h7xx_hal_adc.h) -- ADC3 on those same parts tops out at 12-bit and
        // needs Bits6_12 passed explicitly from the .repl using this peripheral, or a
        // firmware-programmed RES=0 (12-bit) gets misread as 16-bit here.
        public STM32H7_ADC(IMachine machine, double referenceVoltage, uint externalEventFrequency, int dmaChannel = 0,
            IDMA dmaPeripheral = null, ResolutionRange resolutionRange = ResolutionRange.Bits8_16)
            : base(
                machine,
                referenceVoltage,
                externalEventFrequency,
                dmaChannel,
                dmaPeripheral,
                // Base class configuration
                watchdogCount: 3,
                hasCalibration: true,
                channelCount: 19,
                hasPrescaler: true,
                hasVbatPin: true,
                hasChannelSelect: false,
                hasChannelSequence: true,
                hasPowerRegister: false,
                hasOffset: true,
                hasDifferentialMode: true,
                samplingTime: SamplingTime.PerChannel,
                dualMode: true,
                hasLinearityCalibration: true,
                hasChannelInjection: true,
                hasSeparateThresholdRegisters: true,
                resolutionRange: resolutionRange,
                hasChannelPreselection: true,
                hasScanDirection: false
            )
        { }
    }
}
