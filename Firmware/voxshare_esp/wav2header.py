import wave
import sys

def wav_to_24bit_shifted_header(input_wav, output_h, array_name="init_sound"):
    with wave.open(input_wav, 'rb') as wav:
        assert wav.getsampwidth() == 3, "Требуется 24-битный PCM WAV!"
        print(f"Частота: {wav.getframerate()} Hz, Каналы: {wav.getnchannels()}")

        frames = wav.readframes(wav.getnframes())
        samples = []

        for i in range(0, len(frames), 3):
            # Читаем 24-битный сэмпл (little-endian)
            sample_24bit = (frames[i+2] << 16) | (frames[i+1] << 8) | frames[i]
            
            # Расширяем знак до 32 бит (если число отрицательное)
            if sample_24bit & 0x800000:
                sample_32bit = sample_24bit | 0xFF000000  # Дополняем единицами
            else:
                sample_32bit = sample_24bit
            
            # Сдвигаем на 8 бит влево (биты 8-31 = исходные 0-23)
            sample_shifted = (sample_32bit << 8) & 0xFFFFFFFF  # Маска на 32 бита!
            samples.append(sample_shifted)

    with open(output_h, 'w') as f:
        f.write(f"#ifndef {array_name.upper()}_H\n")
        f.write(f"#define {array_name.upper()}_H\n\n")
        f.write(f"const int32_t {array_name}[] = {{\n")
        
        # Записываем по 6 значений в строку для читаемости
        for i in range(0, len(samples), 6):
            batch = samples[i:i+6]
            line = "    " + ", ".join(f"0x{x:08X}" for x in batch) + ","
            f.write(line + "\n")
        
        f.write("};\n")
        f.write(f"const unsigned int {array_name}_length = {len(samples)};\n\n")
        f.write("#endif\n")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Использование: python wav2header.py input.wav output.h")
        sys.exit(1)
    wav_to_24bit_shifted_header(sys.argv[1], sys.argv[2])