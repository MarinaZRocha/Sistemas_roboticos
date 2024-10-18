#!/usr/bin/env python

import cv2
import numpy as np
import os
import glob

# Função para criar diretórios se não existirem
def create_directory(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

# Definindo as dimensões do tabuleiro de xadrez
CHECKERBOARD = (6, 4)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Criando vetores para armazenar pontos 3D e 2D
objpoints = []
imgpoints = []

# Definindo as coordenadas do mundo para os pontos 3D
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

# Extraindo o caminho das imagens
images = glob.glob('/home/marina/sist_robotics/Sistemas_roboticos/perception/dataset_iPad/*.jpg')

if len(images) == 0:
    print("Nenhuma imagem encontrada. Verifique o caminho das imagens.")
else:
    print(f"{len(images)} imagens encontradas.")

# Inicializando o contador para salvar as imagens
num = 0

# Diretórios para salvar as imagens distorcidas e corrigidas
distorcida_dir = '/home/marina/sist_robotics/Sistemas_roboticos/perception/distorcida/'
corrigida_dir = '/home/marina/sist_robotics/Sistemas_roboticos/perception/corrigida/'

# Criando os diretórios se não existirem
create_directory(distorcida_dir)
create_directory(corrigida_dir)

for i, fname in enumerate(images):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Encontrar os cantos do tabuleiro de xadrez
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        objpoints.append(objp)
        # Refinando as coordenadas dos pontos 2D
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Desenhar e exibir os cantos
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

    # Ajustar a janela para se adaptar à imagem
    cv2.namedWindow('img', cv2.WINDOW_NORMAL)
    cv2.imshow('img', img)
    cv2.waitKey(0)

cv2.destroyAllWindows()

# Verificação se as imagens foram encontradas
if len(objpoints) == 0 or len(imgpoints) == 0:
    print("Erro: Nenhuma imagem válida encontrada para calibração.")
else:
    # Execução da calibração da câmera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print(f"Calibração realizada com sucesso. Erro médio: {ret}")

    print("Matriz da câmera: \n", mtx)
    print("Coeficientes de distorção: \n", dist)

    # Gerar imagens mostrando a distorção e a correção
    for i, fname in enumerate(images):
        img = cv2.imread(fname)

        # Corrigir a distorção da imagem
        h, w = img.shape[:2]
        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0.8, (w, h))  # Ajuste o valor de zoom aqui (0.8)
        undistorted_img = cv2.undistort(img, mtx, dist, None, new_camera_mtx)

        # Aplicar a região de interesse (ROI) para cortar as bordas indesejadas
        x, y, w, h = roi
        undistorted_img = undistorted_img[y:y+h, x:x+w]  # Cortar a imagem

        # Salvar a imagem original (com distorção) e a imagem corrigida (sem distorção)
        distorcida_path = os.path.join(distorcida_dir, f'img{num}_distorcida.png')
        corrigida_path = os.path.join(corrigida_dir, f'img{num}_corrigida.png')
        
        cv2.imwrite(distorcida_path, img)
        cv2.imwrite(corrigida_path, undistorted_img)
        
        print(f"Imagens distorcida e corrigida salvas como img{num}.png")
        num += 1

    # Salvar os dados da calibração em um arquivo .YAML
    file_name = "/home/marina/sist_robotics/Sistemas_roboticos/perception/calib.yaml"
    fs = cv2.FileStorage(file_name, cv2.FILE_STORAGE_WRITE)

    # Escrevendo os dados no arquivo
    fs.write("mtx", mtx)
    fs.write("dist", dist)
    fs.write("err", ret)

    fs.release()  # Fechar o arquivo

    print(f"Dados de calibração salvos em {file_name}")