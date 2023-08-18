# VSS-BahiaRT-Testes
VSS BahiaRT ACSO UNEB Simulated 2023
---
#### Repositório de testes do BahiaRT Team.

### Requisitos

Para a instalação, você irá precisar:

**1. Ubuntu versão 20.04**

**2. Dependences:**
```
Python 3.9.17
Protobuf
Numpy
Pyserial
```

### Como instalar

Instale as dependências executando o seguinte comando:
```shell
./InstallDependencies
```

### Como usar

Crie um arquivo *.py* baseado no *template.py* e modifique de acordo com o seu objetivo.

Para escoher entre executar para o ambiente real ou simulado você deverá utilizar o argumento *--env*.

Na pasta raiz do repositório, execute o seu arquivo para o ambiente simulado com seguinte comando:
```shell
python3 nome_do_arquivo.py --env simulation
```
Para o ambiente real, o seguinte comando:
```shell
python3 nome_do_arquivo.py --env real
```


