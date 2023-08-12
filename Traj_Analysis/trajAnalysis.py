f = open("gps1.txt", "r")
filetxt = f.read()

fileData = [x.split('\t') for x in filetxt.split('\n')]

print(fileData[0][2])

'''
for x in fileData:
    print(x,'\n')
'''
    
# check for any times that 
prev = fileData[0][2]
for i in range(len(fileData)-1):
    if fileData[i][2] < prev:
        print(fileData[i][1])
    prev = fileData[i][2]

'''
for x in fileData:
    for y in x:
        print(y, '\t')
    print('\n')
'''

