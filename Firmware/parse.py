
import re 
content = [] 
with open("sixtyDegreeTimes.txt", "r") as f:
    content = f.readlines()

i = 2000
j = 0
avg = 0
with open("sixtyDegreeTimesMod.txt", "w") as f:
    for line in content:
        if (line == "Time\n"):
            line = "Time = " + str(i) + "\n"
            i -= 10
            f.write(line)
        if ("[i]" in line):
            temp = re.findall(r'\d+', line)
            avg += int(temp[0])
            line = line.replace("sixtyDegreeTimesFlag[i]=", str(j) + " = ")
            f.write(line)
            j += 1
            if (j == 6):
                j = 0
                avg //= 6
                f.write("avg = " + str(avg) + "\n")
                f.write("avg% = " + str(round((i - avg)/avg * 100, 2)) + "\n\n")
                avg = 0
