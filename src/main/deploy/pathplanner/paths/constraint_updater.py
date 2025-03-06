import os
import json

def update_path_files():
    # Get user input for constraints
    print("Enter new values for global constraints (-1 to keep existing value)")
    max_velocity = float(input("Max Velocity: "))
    max_acceleration = float(input("Max Acceleration: "))
    max_angular_velocity = float(input("Max Angular Velocity: "))
    max_angular_acceleration = float(input("Max Angular Acceleration: "))

    # Get all .path files in current directory
    path_files = [f for f in os.listdir('.') if f.endswith('.path')]

    if not path_files:
        print("No .path files found in current directory.")
        return

    files_updated = 0

    # Process each file
    for path_file in path_files:
        try:
            # Read the file
            with open(path_file, 'r') as f:
                data = json.load(f)

            # Update global constraints if needed
            constraints = data.get('globalConstraints', {})

            if max_velocity != -1:
                constraints['maxVelocity'] = max_velocity

            if max_acceleration != -1:
                constraints['maxAcceleration'] = max_acceleration

            if max_angular_velocity != -1:
                constraints['maxAngularVelocity'] = max_angular_velocity

            if max_angular_acceleration != -1:
                constraints['maxAngularAcceleration'] = max_angular_acceleration

            # Save the updated data back to file
            with open(path_file, 'w') as f:
                json.dump(data, f, indent=2)

            files_updated += 1
            print(f"Updated {path_file}")

        except Exception as e:
            print(f"Error processing {path_file}: {str(e)}")

    print(f"\nCompleted! Updated {files_updated} out of {len(path_files)} path files.")

if __name__ == "__main__":
    update_path_files()