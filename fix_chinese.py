import os

root_dir = r'd:\develop\Workplace\VscodeWorkplace\stm32\smart_car\Core\App'

fixed_count = 0
for root, dirs, files in os.walk(root_dir):
    for file in files:
        if file.endswith(('.c', '.h')):
            filepath = os.path.join(root, file)
            with open(filepath, 'rb') as f:
                raw_data = f.read()
            
            has_bom = raw_data[:3] == b'\xef\xbb\xbf'
            if has_bom:
                raw_data = raw_data[3:]
            
            has_high_bytes = any(b > 127 for b in raw_data)
            
            if has_high_bytes:
                try:
                    content_utf8 = raw_data.decode('utf-8')
                except:
                    try:
                        content_gbk = raw_data.decode('gbk')
                        content_utf8 = content_gbk.encode('utf-8').decode('utf-8')
                        with open(filepath, 'wb') as f:
                            f.write(content_utf8.encode('utf-8'))
                        print(f"Fixed GBK -> UTF-8: {filepath}")
                        fixed_count += 1
                    except:
                        print(f"Unknown encoding: {filepath}")

print(f"\nTotal fixed: {fixed_count} files")
