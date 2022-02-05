from torch import nn


class Yofo(nn.Module):
    """
    Yofo model
        YO, Find Orientation
    """

    def __init__(self):
        super(Yofo, self).__init__()
        self.deeply_connected = nn.Sequential(
            # input
            nn.BatchNorm2d(1),
            nn.Dropout2d(p=0.05),
            nn.Conv2d(1, 64, kernel_size=(3, 3), stride=(1, 1), padding=1),

            # /2
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=(2, 2), stride=(2, 2), padding=0),
            nn.BatchNorm2d(64),
            nn.Dropout2d(p=0.05),
            nn.Conv2d(64, 64, kernel_size=(3, 3), stride=(1, 1), padding=1),

            # /2
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=(2, 2), stride=(2, 2), padding=0),
            nn.BatchNorm2d(64),
            nn.Dropout2d(p=0.05),
            nn.Conv2d(64, 16, kernel_size=(3, 3), stride=(1, 1), padding=1),

            # flatten
            nn.ReLU(),
            nn.BatchNorm2d(16),
            nn.Dropout2d(p=0.3),
            nn.Flatten(),
            nn.Linear(8*8*16, 32),

            # deep
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.Linear(32, 6),

            # output
            nn.Softmax(1)
        )
        self.float()

    def forward(self, x):
        x = x.transpose(1, 3)  # convert to (batch_size, channels, height, width)
        x = self.deeply_connected(x)
        return x
