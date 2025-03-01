import math

def parse_pose2d(text):
    translation_part = text.split("Translation2d")[1].split(")")[0] + ")"
    rotation_part = text.split("Rotation2d")[1].split("))")[0] + ")"

    x_val = float(translation_part.split("X: ")[1].split(",")[0])
    y_val = float(translation_part.split("Y: ")[1].split(")")[0])

    rads = float(rotation_part.split("Rads: ")[1].split(",")[0])
    degs = float(rotation_part.split("Deg: ")[1].split(")")[0])

    return {
        "x": x_val,
        "y": y_val,
        "rads": rads,
        "degs": degs
    }

def shift_pose(pose, amount):
    # Get the direction vector from the rotation angle
    direction_x = math.cos(pose["rads"])
    direction_y = math.sin(pose["rads"])

    # Calculate the new position
    new_x = pose["x"] + direction_x * amount
    new_y = pose["y"] + direction_y * amount

    return (new_x, new_y, pose["rads"], pose["degs"])

def get_float_input(prompt_text):
    while True:
        try:
            return float(input(prompt_text))
        except ValueError:
            print("Invalid input. Please enter a number.")

def main():
    pose_definitions = [
        "Pose2d(Translation2d(X: 3.66, Y: 4.03), Rotation2d(Rads: 3.14, Deg: 180.00))",
        "Pose2d(Translation2d(X: 4.07, Y: 4.75), Rotation2d(Rads: 2.09, Deg: 120.00))",
        "Pose2d(Translation2d(X: 4.90, Y: 4.75), Rotation2d(Rads: 1.05, Deg: 60.00))",
        "Pose2d(Translation2d(X: 5.32, Y: 4.03), Rotation2d(Rads: 0.00, Deg: 0.00))",
        "Pose2d(Translation2d(X: 4.90, Y: 3.31), Rotation2d(Rads: -1.05, Deg: -60.00))",
        "Pose2d(Translation2d(X: 4.07, Y: 3.31), Rotation2d(Rads: -2.09, Deg: -120.00))"
    ]

    poses = [parse_pose2d(pose) for pose in pose_definitions]

    for i, pose in enumerate(poses):
        print(f"ID {i}: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['degs']:.0f}°)")

    amount = get_float_input("Enter shift amount for all positions: ")

    print("\nShifted positions:")
    for i, pose in enumerate(poses):
        new_position = shift_pose(pose, amount)
        print(f"Z{i} ({new_position[0]:.2f}, {new_position[1]:.2f})")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled")
    except Exception as e:
        print(f"\nError: {e}")