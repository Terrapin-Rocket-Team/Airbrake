Import("env")
import os

def fix_odrive_bug(source, target, env):
    """Fix the ODriveArduino DEBUG bug by patching the source file"""

    # Find the ODriveCAN.cpp file in libdeps
    libdeps_dir = env.subst("$PROJECT_LIBDEPS_DIR")

    for root, dirs, files in os.walk(libdeps_dir):
        if "ODriveArduino" in root and "ODriveCAN.cpp" in files:
            odrive_can_path = os.path.join(root, "ODriveCAN.cpp")

            # Read the file
            with open(odrive_can_path, 'r') as f:
                content = f.read()

            # Replace msg.data with data
            if "msg.data[byte_index--]" in content:
                print(f"Patching ODriveCAN.cpp to fix DEBUG bug...")
                content = content.replace("msg.data[byte_index--]", "data[byte_index--]")

                # Write back
                with open(odrive_can_path, 'w') as f:
                    f.write(content)
                print(f"Successfully patched {odrive_can_path}")
            break

# Run the fix before building
env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", fix_odrive_bug)
