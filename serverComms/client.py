from socket import *
serverName = '127.0.0.1'
serverPort = 12001
clientSocket = socket(AF_INET, SOCK_DGRAM)
message = input('Input number of robots: ')
clientSocket.sendto(message.encode(), (serverName, serverPort))
modMessage, serverAddress = clientSocket.recvfrom(2048)
print('From server: ', modMessage.decode())
clientSocket.close()