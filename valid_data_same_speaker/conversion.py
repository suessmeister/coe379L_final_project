import os
import subprocess

folders = ["hey", "hi", "hello"]

for folder in folders:
    folder_path = os.path.join(os.getcwd(), folder)
    for filename in os.listdir(folder_path):
        if filename.endswith(".m4a"):
            wav_filename = filename.replace(".m4a", ".wav")
            wav_path = os.path.join(folder_path, wav_filename)
            
            # Skip if .wav already exists
            if os.path.exists(wav_path):
                print(f"Skipping {filename}, .wav already exists.")
                continue
            
            m4a_path = os.path.join(folder_path, filename)
            print(f"Converting {filename} to {wav_filename}...")
            
            # Call ffmpeg
            subprocess.run([
                "ffmpeg",
                "-i", m4a_path,
                "-ar", "16000",    # optional: set sample rate to 16kHz
                "-ac", "1",        # optional: convert to mono
                wav_path
            ])
