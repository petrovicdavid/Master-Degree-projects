import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
import torchvision.datasets as datasets
from torchvision.datasets import ImageFolder
from torch.utils.data import DataLoader
import torchvision.models as models
from torchvision.models import alexnet
from torchsummary import summary
from sklearn.metrics import accuracy_score, confusion_matrix
import matplotlib.pyplot as plt
import numpy as np
from sklearn.model_selection import train_test_split
import scipy.io

# define a custom classifier with 7 classes
class CustomClassifier(nn.Module):
    def __init__(self, num_classes):
        super(CustomClassifier, self).__init__()
        self.fc1 = nn.Linear(in_features=4096, out_features=4096, bias=True)
        self.relu1 = nn.ReLU(inplace=True)
        self.drop1 = nn.Dropout(p=0.5, inplace=False)
        self.fc2 = nn.Linear(in_features=4096, out_features=4096, bias=True)
        self.relu2 = nn.ReLU(inplace=True)
        self.fc3 = nn.Linear(in_features=4096, out_features=7, bias=True)
        
    
    def forward(self, x):
        x = x.view(x.size(0), -1)
        x = self.fc1(x)
        x = self.relu1(x)
        x = self.drop1(x)
        x = self.fc2(x)
        x = self.relu2(x)
        x = self.fc3(x)
        return x

# upload AlexNet model
model = models.alexnet(pretrained=True)

# erase last layer
model.classifier = nn.Sequential(
    *list(model.classifier.children())[:-1]  # Rimuove l'ultimo layer
)

# add the new classifier
model.classifier[-1] = CustomClassifier(num_classes=7)


print(model)

# initialization of the parameters

pathP = 'ForaminiferaColored'
batch_size = 30
learning_rate = 1e-4
num_epochs = 50
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


transform = transforms.Compose([
    transforms.Resize((227, 227)),
    transforms.ToTensor()
])

# upload dataset
dataset = ImageFolder(pathP, transform=transform)

# divide the dataset in train and test
train_indices, test_indices = train_test_split(
    range(len(dataset)),
    test_size=0.2,
    stratify=dataset.targets
)


train_dataset = torch.utils.data.Subset(dataset, train_indices)
test_dataset = torch.utils.data.Subset(dataset, test_indices)

num_classes = len(train_dataset.dataset.classes)


train_loader = DataLoader(
    train_dataset,
    batch_size=batch_size,
    shuffle=True
)

test_loader = DataLoader(
    test_dataset,
    batch_size=batch_size,
    shuffle=False
)

model.to(device)

criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD([
        {'params': model.classifier.parameters(), 'lr': 20*learning_rate}
    ], lr=learning_rate, momentum=0.9)

# training
for epoch in range(num_epochs):
    train_correct = 0
    train_total = 0
    for images, labels in train_loader:
        images = images.to(device)
        labels = labels.to(device)

        # forward pass
        outputs = model(images)
        loss = criterion(outputs, labels)

        # backward pass and optimization
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        # calculate accuracy
        _, predicted = torch.max(outputs.data, 1)
        
        train_total += labels.size(0)
        train_correct += (predicted == labels).sum().item()

    train_accuracy = 100 * train_correct / train_total

    print(f'Epoch [{epoch + 1}/{num_epochs}], Loss: {loss.item():.4f}, Train Accuracy: {train_accuracy:.2f}%')

# model evaluation
model.eval()
test_correct = 0
test_total = 0
test_predictions = []
test_labels = []

with torch.no_grad():
    for images, labels in test_loader:
        images = images.to(device)
        labels = labels.to(device)

        outputs = model(images)
        _, predicted = torch.max(outputs.data, 1)
        test_total += labels.size(0)
        test_correct += (predicted == labels).sum().item()

        test_predictions.extend(predicted.cpu().numpy())
        test_labels.extend(labels.cpu().numpy())

test_accuracy = 100 * test_correct / test_total
print(f'Test Accuracy: {test_accuracy:.2f}%')

# calculate confusion matrix
confusion = confusion_matrix(test_labels, test_predictions)
print("Confusion Matrix:")
print(confusion)

# plot results
plt.figure(figsize=(10, 8))
plt.imshow(confusion, cmap=plt.cm.Blues)
plt.title("Confusion Matrix")
plt.colorbar()
tick_marks = np.arange(num_classes)
plt.xticks(tick_marks, dataset.classes, rotation=45)
plt.yticks(tick_marks, dataset.classes)
plt.xlabel("Predicted Class")
plt.ylabel("True Class")
plt.show()