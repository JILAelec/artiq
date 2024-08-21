import os
import shutil
import zipfile

# Define source and destination directories
source_dir = "artiq_kc705/nist_ltc"
destination_dir = "archive"

# List of files to be included in the archive
files_to_include = [
    "gateware/top.bit",
    "software/bootloader/bootloader.bin",
    "software/satman/satman.fbi",
    "software/runtime/runtime.fbi"
]

# Create the necessary directory structure in the destination directory
for file in files_to_include:
    dest_path = os.path.join(destination_dir, os.path.dirname(file))
    os.makedirs(dest_path, exist_ok=True)

    # Copy the file to the destination directory
    src_file = os.path.join(source_dir, file)
    if os.path.exists(src_file):
        shutil.copy(src_file, dest_path)

# Create a zip archive of the destination directory
archive_name = "artiq_kc705_nist_ltc_archive.zip"
shutil.make_archive(archive_name.replace('.zip', ''), 'zip', destination_dir)

print(f"Archive created: {archive_name}")