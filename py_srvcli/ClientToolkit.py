def convertFromStringDictToDict(string):
    dict={}
    string=string[1:-1]
    listItem=string.split(", ")
    for item in listItem:
        key,value=item.split(": ")[0],item.split(": ")[1]
        dict[key]=value
    return dict

def convertFromStringListToList(string):
    list=[]
    string=string[1:-1]
    listItem=string.split(", ")
    for item in listItem:
        list.append(float(item))
    return list

