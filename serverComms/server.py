from socket import *
serverPort = 12001
serverSocket = socket(AF_INET, SOCK_DGRAM)
serverSocket.bind(('', serverPort))
print('The server is ready to receive')
while 1:
    message, clientAddress = serverSocket.recvfrom(2048)
    modMessage = message.decode().upper()
    serverSocket.sendto(modMessage.encode(), clientAddress)
    print('From client: ', modMessage)
serverSocket.close()