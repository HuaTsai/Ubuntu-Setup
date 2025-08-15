from pathlib import Path

class Color:
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    ENDC = "\033[0m"

def main():
    """Remove nonexistent paths from @data file"""
    data_file = Path("~/.local/share/z/data").expanduser()
    
    if not data_file.exists():
        print(f"Error: Data file not found {data_file}")
        return
    
    existing_lines = []
    removed_count = 0
    
    with data_file.open('r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            original_line = line
            line_stripped = line.strip()
            
            if not line_stripped:
                existing_lines.append(original_line)
                continue
                
            # Parse path from line
            path_info = line_stripped.split('|')[0].strip()
            path = Path(path_info)
            
            if path.exists():
                existing_lines.append(original_line)
                print(f"{Color.GREEN}Keep line {line_num}: {path_info}{Color.ENDC}")
            else:
                removed_count += 1
                print(f"{Color.RED}Remove line {line_num}: {path_info}{Color.ENDC}")
    
    # Write back to file
    with data_file.open('w', encoding='utf-8') as f:
        f.writelines(existing_lines)
    
    print(f"\n{Color.YELLOW}Cleanup completed:")
    print(f"Removed {removed_count} nonexistent paths")
    print(f"Kept {len(existing_lines)} entries{Color.ENDC}")

if __name__ == "__main__":
    main()
