import matplotlib.pyplot as plt
#516 was unsuccessful 1, successful 2, bite dropped at 3 - banana
#520 successful banana grasp
#521 multiple unsuccessful banana grasps
#525 and 526 are successful apple grabs each
#528 is unsuccessful apple grab at end - first few are successful
#sometimes apple angle also cannot bite 
import numpy as np 

file1 = open('data 05:16:57 09 Dec copy.txt', 'r')
count = 0
y = [] 
while True:
  
    # Get next line from file
    line = file1.readline()
    if line.startswith("data:"):
        # corresponding y axis values
        print(line)
        testline = line
        test1 = testline.replace('data: [','')
        if ']layout:' in test1: #for last line handling
            test2 = test1.replace(']layout:','')
        else:
            test2 = test1.replace(']','')
        print(test2)
        data = [float(x) for x in test2.split(', ')]
        print(data)
        avg = sum(data) / len(data)
        count += 1
        y.insert(count, int(avg))

    # if line is empty
    # end of file is reached
    if not line:
        break
 #   print("Line{}: {}".format(count, line.strip()))
  
# plotting the points 
x = list(range(1, count+1))

np.save('00002',y)
arr = 0
np.save('lab',arr)
print(x)
print(y)

plt.plot(x, y)
        
# naming the x axis
plt.xlabel('time')
# naming the y axis
plt.ylabel('touch sensor reading')
    
# giving a title to my graph
plt.title('Touch Sensor Feedback')

plt.savefig('data 05:16:57 09 Dec copy.png')

# function to show the plot
plt.show()


file1.close()