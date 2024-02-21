from distutils.sysconfig import project_base
import torch
import clip
import os
import sys
from PIL import Image

from torchvision import transforms
from torch.nn.functional import normalize, softmax



# write the labels not already re-detected in a file
def rewriteFile(labels, ris):
    global projectPath
    f = open(projectPath+"/data/img/labels.txt" , "w")
    l = [i[0] for i in ris]
    for i in labels:
        if i not in l:
            f.write(str(i+1) + '\n')
    f.close()

 
 
 # check if the constraints are satisfied
def checkPrediction(values, indices):
    global mode, numberPlates

    first_plates = [0,1,2,3,4]
    second_plates = [5,6,7,8]
    third_plates = [9,10]
    others = [11,12]
    # list of tuple (label, probability)
    prob = [ (indices[i], values[i].item()) for i in range(len(indices))]
    
    # sort the list based on the probabilities of the tuple in decrescent mode 
    prob.sort(key=lambda p: p[1], reverse=True)
    
    
    top = prob[0]
    
    # food_image mode
    if mode == 0:
        # if the food is of type first plate, return one prediction
        if(top[0] in first_plates):
            return [top]

        # if the food is of type second plate, return at most three prediction (second plate + at most two third plates)
        ris = []
        contThird = 0
        contSecond = 0

        if(top[0] in second_plates):
            ris.append(top)
            for i in range(1, len(prob)):
                tupla = prob[i]
                if(tupla[0] in third_plates):
                    if(tupla[1] > 0.02 and contThird < 2):
                        contThird += 1
                        ris.append(tupla)
            return ris

        # if the food is of type third plate, return at most three prediction (second plate + at most two third plate)
        if(top[0] in third_plates):
            ris.append(top)
            contThird += 1
            for i in range(1, len(prob)):
                tupla = prob[i]
                if(tupla[0] in second_plates and contSecond == 0):
                    contSecond += 1
                    ris.append(tupla)

                if(tupla[0] in third_plates):
                    if(tupla[1] > 0.02 and contThird < 2):
                        contThird += 1
                        ris.append(tupla)

            return ris
        
        if(top[0] in others):
            return [0,0]


    # leftover image mode
    if mode == 1:
        
        # get the labels not already re-assigned 
        f = open(projectPath+ "/data/img/labels.txt", "r")
        lines = []
        for i in range(numberPlates+1):
            lines.append(f.readline())
        f.close()
        
        # convert the labels in integer and set them for starting with index 0
        labels = [int(i)-1 for i in lines if i != '']

        ris = []
        contThird = 0
        contSecond = 0

        # go through all the labels
        for i in range(len(prob)):
            # if the food is of type first plate and the corresponding label was assigned in the food_image, return one prediction
            # the labels previously assigned in the food_image are send by the c++ software
            if(prob[i][0] in first_plates and prob[i][0] in labels):
                if prob[i][1] > 0.03:
                    ris.append(prob[i])
                    rewriteFile(labels, ris)
                    return ris

            # if the food is of type second plate and the corresponding label was assigned in the food_image, return at most three prediction (second plate + at most two third plate)
            if(prob[i][0] in second_plates and prob[i][0] in labels):
                if prob[i][1] > 0.03:
                    ris.append(prob[i])
                    contSecond += 1
                else:
                    return [0,0]

                # go through the remaining labels for searching the third plates
                for j in range(i+1, len(prob)):
                    tupla = prob[j]
                    if(tupla[0] in third_plates and tupla[0] in labels):
                        if(tupla[1] > 0.03 and contThird < 2):
                            contThird += 1
                            ris.append(tupla)
                        else:
                            return [0,0]
                rewriteFile(labels, ris)
                return ris


            # if the food is of type third plate and the corresponding label was assigned in the food_image, return at most three prediction (second plate + at most two third plate)
            if(prob[i][0] in third_plates and prob[i][0] in labels and contThird<2):
                if prob[i][1] > 0.03:
                    ris.append(prob[i])
                    contThird += 1
                else:
                    return [0,0]

                # go through the remaining labels for searching the second and the (possibly) third plates 
                for j in range(i+1, len(prob)):
                    tupla = prob[j]
                    if(tupla[0] in second_plates and contSecond == 0 and tupla[0] in labels):
                        if(tupla[1] > 0.03):
                            contSecond += 1
                            ris.append(tupla)
                        else:
                            return [0,0]

                    if(tupla[0] in third_plates and tupla[0] in labels):
                        if(tupla[1] > 0.03 and contThird < 2):
                            contThird += 1
                            ris.append(tupla)
                        else:
                            return [0,0]
                            
                rewriteFile(labels, ris)
                return ris

        return [0,0]
    


# write the predictions in a file
def writeResult(ris):
    global projectPath
    f = open(projectPath+"/data/img/output.txt", 'w')
    if ris != [-1,-1] and ris != [0,0]:
        for i in range(len(ris)):
            f.write(str(ris[i][0]+1)+'\n')
    else:
        f.write(str(-1)+'\n')
    
    f.close()



# make prediction, check the result and write it in the file
def detect(image, labels):
    global input_folder, output_folder
    global device, model, preprocess

    img = Image.open(image)
    image = preprocess(img).unsqueeze(0).to(device)
    text = clip.tokenize(labels).to(device)

    with torch.no_grad():
        image_features = model.encode_image(image)
        text_features = model.encode_text(text)

    image_features_norm = normalize(image_features, p=2, dim=-1)
    text_features_norm = normalize(text_features, p=2, dim=-1)
    similarity = (100.0 * torch.matmul(image_features_norm, text_features_norm.T))
    similarity = softmax(similarity, dim=-1)
    
    values = []
    indices = []
    for i in range(len(labels)):
        values.append(similarity[0][i])
        indices.append(i)

    ris = checkPrediction(values, indices)

    writeResult(ris)



def main( i : int = None ):

    global device, model, preprocess, input_folder, output_folder, mode, numberPlates, projectPath



    # mode = 0 => food_image as input
    # mode = 1 => leftover image as input
    mode = int(sys.argv[1])
    numberPlates = int(sys.argv[2])
    projectPath = sys.argv[3]

    
    image = projectPath+'/data/img/img.jpg'
    output_folder = projectPath+'/data/img/output/'

    labels = [
        "pasta pesto",
        "pasta with tomato",
        "pasta bolognese",
        "pasta with fish sauce",
        "pilaw rice with peppers and peas",
        "pork steak",
        "fish cutlet",
        "rabbit meat",
        "cuttlefish food",
        "brown beans",
        "potatoes"
    ]

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, preprocess = clip.load("ViT-B/32", device=device)

    detect(image, labels)


    
if __name__ == "__main__":
    print ("Python script started")
    main()
    print ("Python ended")
