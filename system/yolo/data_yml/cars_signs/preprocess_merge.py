import os
import csv
import yaml
import shutil
from pathlib import Path
import json
from tqdm import tqdm

def preprocess_yolo(datasets_config, output_dir, class_merge_map):
    """
    datasets_config: List of dicts with original dataset paths and their specific ID maps.
    output_dir: Where the output will be saved.
    class_merge_map: Dict { 'original_name': 'merged_name' }
                     e.g., {'car': 'vehicle', 'van': 'vehicle', 'pedestrian': 'person'}
    """
    output_path = Path(output_dir)
    # Create the top level and the labels sub-structure
    (output_path / "labels" / "train").mkdir(parents=True, exist_ok=True)
    (output_path / "labels" / "val").mkdir(parents=True, exist_ok=True)
    (output_path / "images" / "train").mkdir(parents=True, exist_ok=True)
    (output_path / "images" / "val").mkdir(parents=True, exist_ok=True)
    
    # 1. Establish unified IDs
    unique_target_names = sorted(list(set(class_merge_map.values())))
    new_class_map = {name: i for i, name in enumerate(unique_target_names)}
    
    print(f"Final Unified Mapping (ID -> Name):")
    for name, idx in new_class_map.items():
        print(f"  {idx}: {name}")

    for split in ['train', 'val']:
        manifest_entries = []
        # Target folder for labels in this specific split
        target_label_dir = output_path / "labels" / split
        target_image_dir = output_path / "images" / split
        for ds in datasets_config:
            ds_root = Path(ds['path'])

            # Note: Checking both common YOLO structures (labels/split or split/labels)
            label_dir = ds_root / 'labels' / split
            image_dir = ds_root / 'images' / split
            
            # 2. Build a remap for this specific dataset
            id_remap = {}
            for old_id, original_name in ds['classes'].items():
                # 1. Check if we actually want this class (is it in our merge map?)
                if original_name in class_merge_map:
                    merged_name = class_merge_map[original_name]
                    
                    # 2. Check if the 'target' name exists in our new ID mapping
                    if merged_name in new_class_map:
                        id_remap[int(old_id)] = new_class_map[merged_name]
                        
            if not label_dir.exists():
                print(f"Warning: {label_dir} not found. Skipping.")
                continue

            for label_file in tqdm(label_dir.glob('*.txt'), desc=f'Processing {split} in {ds["path"]}'):
                valid_lines = []
                with open(label_file, 'r') as f:
                    for line in f:
                        parts = line.split()
                        if not parts: continue
                        old_id = int(parts[0])
                        
                        if old_id in id_remap:
                            parts[0] = str(id_remap[old_id])
                            valid_lines.append(" ".join(parts))
                
                if valid_lines:
                    # Find corresponding image
                    img_path = None
                    for ext in ['.jpg', '.jpeg', '.png']:
                        potential_img = image_dir / f"{label_file.stem}{ext}"
                        if potential_img.exists():
                            img_path = potential_img
                            break
                    
                    if img_path:
                        # 3. Save UPDATED label file to the CENTRALIZED output_dir
                        # We use the dataset name as a prefix to prevent filename collisions
                        # ds_prefix = ds_root.stem
                        # new_label_name = f"{ds_prefix}_{label_file.name}"
                        new_label_name = label_file.name
                        with open(target_label_dir / new_label_name, 'w') as nf:
                            nf.write("\n".join(valid_lines))
                        
                        # 4. Copy image to the CENTRALIZED output_dir
                        new_img_name = img_path.name
                        shutil.copy2(img_path, target_image_dir / new_img_name)
                        
                        # 5. Write manifest entry
                        manifest_entries.append(f"./images/{split}/{new_img_name}")
        
        # Write manifest (train.txt or val.txt)
        with open(output_path / f"{split}.txt", 'w') as f:
            f.write("\n".join(manifest_entries))

    print(f"\nDone! Unified dataset ready at: {output_dir}")

##--- CONFIGURATION ---
# Keys = Names found in your datasets
# Values = What you want them to be called (and grouped by) in the final model
    # signs
    # --- STOP SIGNS ---
    # --- TRAFFIC LIGHTS ---
    # --- SPEED LIMITS ---
with open('./class_merge_map.json', "r") as f:
    class_merge_map = json.load(f)

datasets_config = []
for yaml_file in ['./signs.yaml', './cars.yaml']:
    with open(yaml_file, 'r') as f:
        config = yaml.safe_load(f)
        datasets_config.append({'path': config['path'], 'classes': config['names']})  

preprocess_yolo(
    datasets_config=datasets_config, 
    output_dir='/mnt/d/DATASETS/cars_signs', 
    class_merge_map=class_merge_map,
)