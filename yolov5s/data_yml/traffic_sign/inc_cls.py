import os

def increment_yolo_classes(folder_path, n_increment, output_path=None):
    """
    Reads YOLO .txt files and increments the class_id by n_increment.
    """
    # Create output directory if it doesn't exist (if saving to a new place)
    if output_path and not os.path.exists(output_path):
        os.makedirs(output_path)

    # Process each file in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith(".txt") and filename != "classes.txt":
            file_path = os.path.join(folder_path, filename)
            
            with open(file_path, 'r') as f:
                lines = f.readlines()

            new_lines = []
            for line in lines:
                parts = line.split()
                if len(parts) == 0: continue # Skip empty lines
                
                # 1. Increment the class (first element)
                # parts[0] is the class_id, parts[1:] are coordinates
                old_class = int(parts[0])
                new_class = old_class + n_increment
                
                # 2. Rebuild the line string
                new_line = f"{new_class} " + " ".join(parts[1:]) + "\n"
                new_lines.append(new_line)

            # 3. Save the modified file
            save_dest = os.path.join(output_path if output_path else folder_path, filename)
            with open(save_dest, 'w') as f:
                f.writelines(new_lines)

            print(f"Processed: {filename}")

# --- CONFIGURATION ---
input_folder = "path/to/your/labels"  # Folder containing .txt files
n = 5                                 # Amount to increment classes by
out_folder = "path/to/new_labels"     # Set to None to overwrite original files

increment_yolo_classes(input_folder, n, out_folder)