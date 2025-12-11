from copy import deepcopy

test_map_1 = [
    [1,1,1,0,0],
    [0,1,0,0,0],
    [0,0,0,0,0],
    [0,0,0,0,0],
    [0,0,0,0,0],
]

test_map_2 = [
    [0,0,0,0,0],
    [0,0,1,1,0],
    [0,0,1,0,0],
    [0,0,0,1,0],
    [0,0,0,0,0],
]

def naieveFuse(map1, map2):
    returnMap = []
    for row1,row2 in zip(map1,map2):
        returnMap.append(list(map(lambda x, y: x or y, row1, row2)))
    
    #for row in returnMap:
    #    print(row)
    
    return returnMap

def fuse(map1, map2):
    m1Ones = []
    for x in range(len(map1)):
        for y in range(len(map1[x])):
            if map1[x][y] == 1:
                m1Ones.append([x,y])

    m2Ones = []
    for x in range(len(map2)):
        for y in range(len(map2[x])):
            if map2[x][y] == 1:
                m2Ones.append([x,y])

    similarity = [0, [0,0]]
    for m1Coord in m1Ones:
        for m2Coord in m2Ones:
            offset = []
            offset.append(m2Coord[0] - m1Coord[0])
            offset.append(m2Coord[1] - m1Coord[1])
            alignedMap = align(map1, offset)
            sim = 0
            for m1row, m2row in zip(alignedMap,map2):
                for m1col, m2col in zip(m1row,m2row):
                    if m1col == m2col == 1:
                        sim += 1
            if sim > similarity[0]:
                similarity = [sim, offset]

    offset = similarity[1]
    copiedOffset = offset.copy()
    while copiedOffset[0] > 0:
        for row in map1:
            row.insert(0,0)
        for row in map2:
            row.append(0)
        copiedOffset[0] -= 1

    while copiedOffset[0] < 0:
        for row in map2:
            row.insert(0,0)
        for row in map1:
            row.append(0)
        copiedOffset[0] += 1

    while copiedOffset[1] > 0:
        map1.insert(0,[0 for i in range(len(map1[0]))])
        map2.append([0 for i in range(len(map2[0]))])
        copiedOffset[1] -= 1

    while copiedOffset[1] < 0:
        map2.insert(0,[0 for i in range(len(map2[0]))])
        map1.append([0 for i in range(len(map1[0]))])
        copiedOffset[1] += 1

    returnMap = []
    for row1,row2 in zip(map1,map2):
        returnMap.append(list(map(lambda x, y: x or y, row1, row2)))
    
    #shiftToTop(returnMap, offset)

    #print(offset)
    for row in returnMap:
        print(row)
    return returnMap#, offset

def align(inputMap, offset):
    copiedList = deepcopy(inputMap)
    copiedOffset = deepcopy(offset)
    # y,x

    while copiedOffset[1] > 0:
        for row in copiedList:
            row.insert(0,0)
            row.pop()
        copiedOffset[1] -= 1

    while copiedOffset[1] < 0:
        for row in copiedList:
            row.append(0)
            row.pop(0)
        copiedOffset[1] += 1

    while copiedOffset[0] > 0:
        copiedList.insert(0,[0 for i in range(len(copiedList[0]))])
        copiedList.pop()
        copiedOffset[0] -= 1

    while copiedOffset[0] < 0:
        copiedList.append([0 for i in range(len(copiedList[0]))])
        copiedList.pop(0)
        copiedOffset[0] += 1

    return copiedList

def shiftToTop(inputMap, offset):
    if 1 not in inputMap[0]:
        inputMap.pop(0)
        offset[0] -= 1
        shiftToTop(inputMap, offset)
    allFirstRow = []
    for row in inputMap:
        allFirstRow.append(row[0])
    if 1 not in allFirstRow:
        for row in inputMap:
            row.pop(0)
        offset[1] -= 1
        shiftToTop(inputMap, offset)

if __name__ == "__main__":
    fuse(test_map_1, test_map_2)