import os
import subprocess

# List of folders containing your audio samples
folders = ["hey", "hi", "hello"]

for folder in folders:
    for file in os.listdir(folder):
        if file.endswith(".m4a"):
            m4a_path = os.path.join(folder, file)
            wav_path = os.path.join(folder, file.replace(".m4a", ".wav"))
            
            # Run ffmpeg to convert m4a -> wav
            subprocess.run([
                "ffmpeg",
                "-y",  # overwrite if file exists
                "-i", m4a_path,
                wav_path
            ])
            print(f"Converted {m4a_path} -> {wav_path}")
