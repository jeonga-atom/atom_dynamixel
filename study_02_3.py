import torch                                   # Pytorch 라이브러리 임포트
from torch import nn                           # Pytorch 라이브러리 안에 있는 신경망(nn)관련 기능 임포트
from torch.utils.data import DataLoader        # 데이터 로딩을 위한 Dataloder 임포트
from torchvision import datasets               # torchvision에서 이미지 데이터셋 임포트
from torchvision.transforms import ToTensor    # 이미지를 Tensor로 변환하는 함수 임포트

# FashionMIST 데이터셋 다운로드 및 로드(훈련 데이터)
training_data = datasets.FashionMNIST(
    root="data",            # 데이터를 저장할 경로
    train=True,             # 훈련 데이터셋 로드, train의 True 값은 훈련 데이터셋을 의미
    download=True,          # 데이터가 없으면 다운로드
    transform=ToTensor()    # 데이터를 Tensor로 변환
)

# FashionMIST 데이터셋 다운로드 및 로드(테스트 데이터)
test_data = datasets.FashionMNIST(
    root="data",            # 데이터를 저장할 경로
    train=False,            # 테스트 데이터셋 로드, train의 False 값은 훈련 데이터셋을 의미
    download=True,          # 데이터가 없으면 다운로드
    transform=ToTensor()    # 데이터를 Tensor로 변환
)

train_dataloader = DataLoader(training_data, batch_size=64) # 훈련 데이터 로더 설정, 훈련 데이터, batch 크기 64
test_dataloader = DataLoader(test_data, batch_size=64)      # 테스트 데이터 로더 설정, 테스트 데이터, batch 크기 64

class NeuralNetwork(nn.Module):                 # 신경망 모델 정의, Pytorch의 nn.Module을 받아 신경망으로 정의
    def __init__(self):                             
        super().__init__()                      # 부모 클래스 초기화 
        self.flatten = nn.Flatten()             # 28x28 이미지를 1차원 벡터로 변환하는 레이어
        self.linear_relu_stack = nn.Sequential( # 신경망 구조를 nn.Sequential을 이용해 정의
            nn.Linear(28*28, 512),              # 입력층: 28x28(784) -> 은닉층: 512개 뉴런
            nn.ReLU(),                          # 활성화 함수 (ReLU)
            nn.Linear(512, 512),                # 은닉층: 512 -> 512
            nn.ReLU(),                          # 활성화 함수 (ReLU)
            nn.Linear(512, 10),                 # 출력층: 512 → 10 (10개의 클래스)
        )

    def forward(self, x):                       # 순전파 함수 (입력 x를 받아 출력값 반환)
        x = self.flatten(x)                     # 이미지 데이터를 1차원 벡터로 변환
        logits = self.linear_relu_stack(x)      # 신경망 통과
        return logits                           # 출력 반환

model = NeuralNetwork()     # 신경망 모델 인스턴스 생성

learning_rate = 1e-3        # 학습률
batch_size = 64             # 1 batch에 포함되는 샘플 수
epochs = 5                  # 최대 epochs 수

# Initialize the loss function
loss_fn = nn.CrossEntropyLoss()     # 손실함수를 cross entropy 사용

# 최적화 알고리즘
optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate)
# optimizer로 SGD사용

def train_loop(dataloader, model, loss_fn, optimizer):  # 훈련 루프
    size = len(dataloader.dataset)                      # 전체 데이터셋 크기
    # Set the model to training mode - important for batch normalization and dropout layers
    # Unnecessary in this situation but added for best practices
    model.train()                                       # 모델을 훈련 모드로 설정
    for batch, (X, y) in enumerate(dataloader):         # batch 단위로 데이터 불러오기
        # Compute prediction and loss
        pred = model(X)                                 # 모델 예측값 계산
        loss = loss_fn(pred, y)                         # 손실 함수 계산

        # Backpropagation
        loss.backward()         # loss에 대해서 역전파
        optimizer.step()        # weight 업데이트
        optimizer.zero_grad()   # 미분값 초기화

        if batch % 100 == 0:    # 100번째 batch 마다 손실 출력
            loss, current = loss.item(), batch * batch_size + len(X)    # loss 계산
            print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")


def test_loop(dataloader, model, loss_fn):  # 테스트 루프
    # Set the model to evaluation mode - important for batch normalization and dropout layers
    # Unnecessary in this situation but added for best practices
    model.eval()                        # 모델을 평가 모드로 설정
    size = len(dataloader.dataset)      # 전체 데이터셋 크기
    num_batches = len(dataloader)       # batch 개수
    test_loss, correct = 0, 0           # loss와 correct 저장할 변수 초기화

    # Evaluating the model with torch.no_grad() ensures that no gradients are computed during test mode
    # also serves to reduce unnecessary gradient computations and memory usage for tensors with requires_grad=True
    with torch.no_grad():               # 테스터 데이터에 대해 정확도를 측정
        for X, y in dataloader:         
            pred = model(X)             # 모델 예측값 계산
            test_loss += loss_fn(pred, y).item()    # 손실 누적
            correct += (pred.argmax(1) == y).type(torch.float).sum().item() # correct는 맞춘 개수 계산

    test_loss /= num_batches    # loss 계산
    correct /= size             # correct, 정확도 계산
    print(f"Test Error: \n Accuracy: {(100*correct):>0.1f}%, Avg loss: {test_loss:>8f} \n") # 정확도, 평균 손실 출력

loss_fn = nn.CrossEntropyLoss() # 손실 함수 재설정
optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate)   # SGD 최적화 알고리즘 설정

epochs = 10     # 훈련을 10번 반복
for t in range(epochs):     # epochs 횟수만큼 반복
    print(f"Epoch {t+1}\n-------------------------------")  # 현재 epoch 번호 출력
    train_loop(train_dataloader, model, loss_fn, optimizer) # 1 epoch를 수행
    test_loop(test_dataloader, model, loss_fn)  # 테스트 데이터 정확도 측정, 테스트 루프 실행
print("Done!")     # 학습 완ㄹ료 메시지 출력
