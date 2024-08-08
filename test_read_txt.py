def read_txt_file(output, file_name):
    with open(file_name, "r") as f:
        num = 0
        oytput = []
        for line in f.readlines():
            line.strip('\n')
            # print(line)
            # print(type(line))
            listline = line.split(" ")
            output.append([])
            for item in listline:
                temp = float(item)
                # temp = 2048 + temp * (2048/3.1415926)
                output[num].append(temp)

            num += 1

        # print(table)

def read_txt_file_2(output, file_name):
    with open(file_name, "r") as f:
        num = 0
        oytput = []
        for line in f.readlines():
            line.strip('\n')
            # print(line)
            # print(type(line))
            listline = line.split(",")
            output.append([])
            for item in listline:
                temp = float(item)
                temp = 2048 + temp * (2048/3.1415926)
                output[num].append(int(temp))

            num += 1

        # print(table)

if __name__ == '__main__':
    table = []
    read_txt_file(table, "test_read_txt.txt")
    print(table)