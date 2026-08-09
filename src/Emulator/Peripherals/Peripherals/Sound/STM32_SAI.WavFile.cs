//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
// Minimal PCM WAV reader/writer used by STM32_SAI to inject/capture audio
// content. Deliberately narrow: integer PCM only (8/16/24/32-bit), no
// float/compressed formats, no multi-'data'-chunk files. That covers what
// any mainstream audio tool exports and is all STM32_SAI needs.
using System;
using System.IO;

namespace Antmicro.Renode.Peripherals.Sound
{
    // Reads one interleaved-sample-at-a-time, upconverting to a left-justified 32-bit value
    // (e.g. a 16-bit source sample 0x1234 becomes 0x12340000) regardless of the file's own bit
    // depth, and loops back to the start of the audio data once exhausted so it can feed a
    // simulation that runs for longer than the source file's duration.
    public class WavPcm32Reader : IDisposable
    {
        public WavPcm32Reader(string path)
        {
            stream = new FileStream(path, FileMode.Open, FileAccess.Read);
            reader = new BinaryReader(stream);
            ParseHeader();
        }

        public uint NextSample()
        {
            if(dataLength == 0)
            {
                return 0;
            }
            if(stream.Position >= dataStart + dataLength)
            {
                stream.Position = dataStart;
            }

            switch(bitsPerSample)
            {
            case 8:
                // 8-bit PCM is conventionally unsigned with a 0x80 midpoint; recenter before upconverting.
                return (uint)((reader.ReadByte() - 0x80) << 24);
            case 16:
                return (uint)((short)reader.ReadUInt16() << 16);
            case 24:
            {
                var b0 = reader.ReadByte();
                var b1 = reader.ReadByte();
                var b2 = reader.ReadByte();
                var value = b0 | (b1 << 8) | (b2 << 16);
                // Sign-extend the 24-bit value before shifting into the top 24 bits of the word.
                if((value & 0x800000) != 0)
                {
                    value |= unchecked((int)0xFF000000);
                }
                return unchecked((uint)(value << 8));
            }
            case 32:
                return reader.ReadUInt32();
            default:
                return 0;
            }
        }

        public void Dispose()
        {
            reader.Dispose();
            stream.Dispose();
        }

        private void ParseHeader()
        {
            if(new string(reader.ReadChars(4)) != "RIFF")
            {
                throw new InvalidDataException("Not a RIFF file");
            }
            reader.ReadUInt32(); // RIFF chunk size, unused
            if(new string(reader.ReadChars(4)) != "WAVE")
            {
                throw new InvalidDataException("Not a WAVE file");
            }

            while(stream.Position < stream.Length)
            {
                var chunkId = new string(reader.ReadChars(4));
                var chunkSize = reader.ReadUInt32();
                var chunkStart = stream.Position;

                if(chunkId == "fmt ")
                {
                    reader.ReadUInt16(); // audio format, assumed to be integer PCM
                    channels = reader.ReadUInt16();
                    reader.ReadUInt32(); // sample rate, informational only for a reader
                    reader.ReadUInt32(); // byte rate
                    reader.ReadUInt16(); // block align
                    bitsPerSample = reader.ReadUInt16();
                }
                else if(chunkId == "data")
                {
                    dataStart = stream.Position;
                    dataLength = chunkSize;
                }

                // RIFF chunks are word-aligned; skip padding if the declared size is odd.
                stream.Position = chunkStart + chunkSize + (chunkSize % 2);
            }

            if(dataLength == 0 || bitsPerSample == 0)
            {
                throw new InvalidDataException("WAV file has no usable 'fmt '/'data' chunks");
            }
            stream.Position = dataStart;
        }

        private readonly FileStream stream;
        private readonly BinaryReader reader;
        private int channels;
        private int bitsPerSample;
        private long dataStart;
        private long dataLength;
    }

    // Writes one interleaved 32-bit-PCM sample at a time (the raw value straight off the bus, no
    // reinterpretation), buffering the header until Dispose() so the RIFF/data sizes can be
    // patched in with the final byte count.
    public class WavPcm32Writer : IDisposable
    {
        public WavPcm32Writer(string path, uint sampleRate, uint channels)
        {
            this.channels = channels;
            stream = new FileStream(path, FileMode.Create, FileAccess.Write);
            writer = new BinaryWriter(stream);
            WriteHeader(sampleRate, channels);
        }

        public void WriteSample(uint value)
        {
            writer.Write(value);
            sampleCount++;
        }

        public void Dispose()
        {
            PatchSizes();
            writer.Dispose();
            stream.Dispose();
        }

        private void WriteHeader(uint sampleRate, uint channels)
        {
            const int bitsPerSample = 32;
            var blockAlign = channels * (bitsPerSample / 8);
            var byteRate = sampleRate * blockAlign;

            writer.Write("RIFF".ToCharArray());
            writer.Write(0u); // patched in Dispose()
            writer.Write("WAVE".ToCharArray());

            writer.Write("fmt ".ToCharArray());
            writer.Write(16u); // fmt chunk size
            writer.Write((ushort)1); // WAVE_FORMAT_PCM
            writer.Write((ushort)channels);
            writer.Write(sampleRate);
            writer.Write(byteRate);
            writer.Write((ushort)blockAlign);
            writer.Write((ushort)bitsPerSample);

            writer.Write("data".ToCharArray());
            writer.Write(0u); // patched in Dispose()
            dataStart = stream.Position;
        }

        private void PatchSizes()
        {
            var dataBytes = (uint)(sampleCount * 4);
            writer.Flush();
            stream.Position = 4;
            writer.Write(36u + dataBytes); // RIFF chunk size = file size - 8
            stream.Position = dataStart - 4;
            writer.Write(dataBytes);
        }

        private readonly FileStream stream;
        private readonly BinaryWriter writer;
        private readonly uint channels;
        private long dataStart;
        private long sampleCount;
    }
}
