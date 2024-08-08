import time

def write_txt_file(filename, a):
    with open(filename, 'a') as f:
        x1, x2, x3 = a
        f.write(f"{x1} {x2} {x3}\n")
        
if __name__ == '__main__':
    for i in range(10):
        write_txt_file("test_write_txt_5.txt", [i, i, i, i])
        # time.sleep(0.5)
        
    print("end")