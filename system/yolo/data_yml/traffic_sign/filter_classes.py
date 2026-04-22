import os
import json
import yaml
from pathlib import Path
from tqdm import tqdm

def filter_yolo_classes(yaml_path, merge_map_path, format_type='split_first'):
    # Load configuration
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
        
    with open(merge_map_path, 'r') as f:
        class_merge_map = json.load(f)

    # Note: path might be Windows style, let's fix it for WSL if needed
    base_path_str = config['path']
    if base_path_str.startswith('D:'):
        base_path_str = base_path_str.replace('D:\\', '/mnt/d/').replace('\\', '/')
    base_path = Path(base_path_str)
    
    original_names = config['names']
    
    # Support both list and dict for names
    if isinstance(original_names, list):
        old_class_map = {i: name for i, name in enumerate(original_names)}
    else:
        old_class_map = {int(k): v for k, v in original_names.items()}

    # 1. Establish unified IDs
    unique_target_names = sorted(list(set(class_merge_map.values())))
    new_class_map = {name: i for i, name in enumerate(unique_target_names)}
    
    print(f"Final Unified Mapping (ID -> Name):")
    for name, idx in new_class_map.items():
        print(f"  {idx}: {name}")

    # Build a remap for this specific dataset
    id_remap = {}
    for old_id, original_name in old_class_map.items():
        if original_name in class_merge_map:
            merged_name = class_merge_map[original_name]
            if merged_name in new_class_map:
                id_remap[old_id] = new_class_map[merged_name]

    # Process splits
    splits = []
    if 'train' in config: splits.append(('train', config['train']))
    if 'val' in config: splits.append(('val', config['val']))
    if 'test' in config: splits.append(('test', config['test']))
    
    for split_name, split_path_str in splits:
        manifest_entries = []
        
        # We deduce the split_folder (e.g. 'train' or 'valid')
        if split_path_str.endswith('.txt'):
            # Fallback if config has train: train.txt
            split_folder = 'valid' if split_name == 'val' else split_name
        else:
            split_folder = split_path_str.replace('\\', '/').split('/')[0] if format_type == 'split_first' else split_path_str.replace('\\', '/').split('/')[-1]
            
        if format_type == 'split_first':
            # e.g. train/images and train/labels
            image_dir = base_path / split_folder / 'images'
            label_dir = base_path / split_folder / 'labels'
            target_label_dir = base_path / split_folder / 'labels_new'
        elif format_type == 'images_first':
            # e.g. images/train and labels/train
            image_dir = base_path / 'images' / split_folder
            label_dir = base_path / 'labels' / split_folder
            target_label_dir = base_path / 'labels' / f"{split_folder}_new"
        else:
            raise ValueError(f"Unknown format_type: {format_type}")
        
        if not label_dir.exists():
            print(f"Warning: Label directory {label_dir} not found. Skipping split: {split_name}.")
            continue
            
        target_label_dir.mkdir(parents=True, exist_ok=True)
        
        for label_file in tqdm(list(label_dir.glob('*.txt')), desc=f'Processing {split_name}'):
            valid_lines = []
            with open(label_file, 'r') as f:
                for line in f:
                    parts = line.split()
                    if not parts: continue
                    try:
                        old_id = int(parts[0])
                    except ValueError:
                        continue
                    
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
                    # Write the new label file
                    with open(target_label_dir / label_file.name, 'w') as nf:
                        nf.write("\n".join(valid_lines) + "\n")
                    
                    # Write manifest entry (using absolute path or relative to dataset root)
                    # YOLO typically works well with absolute paths or paths relative to dataset root
                    rel_img_path = f"./{image_dir.relative_to(base_path).as_posix()}/{img_path.name}"
                    manifest_entries.append(rel_img_path)
                    
        # Write manifest file at the root of the dataset (e.g. train.txt, val.txt)
        manifest_path = base_path / f"{split_name}_new.txt"
        with open(manifest_path, 'w') as f:
            f.write("\n".join(manifest_entries))
            
        print(f"Generated manifest: {manifest_path} with {len(manifest_entries)} images.")

    # Create a new yaml file for the updated dataset
    new_yaml_path = base_path / "traffic_sign_filtered.yaml"
    new_config = {
        'path': str(base_path),
        'train': 'train.txt' if 'train' in config else '',
        'val': 'val.txt' if 'val' in config else '',
        'test': 'test.txt' if 'test' in config else '',
        'nc': len(new_class_map),
        'names': {v: k for k, v in new_class_map.items()}
    }
    with open(new_yaml_path, 'w') as f:
        yaml.dump(new_config, f, sort_keys=False)
        
    print(f"Done! New dataset config saved at: {new_yaml_path}")
    print("Remember to rename 'train_new.txt' to 'train.txt' (and back up original 'train.txt') when training!")
    print("Remember to rename 'labels_new' to 'labels' (and back up original 'labels') when training!")

yaml_file = './traffic_sign.yaml'
merge_map_file = './class_merge_map.json'

# format_type can be 'split_first' (train/images) or 'images_first' (images/train)
filter_yolo_classes(yaml_file, merge_map_file, format_type='split_first')
