from socket import *

def choose_server_port(server_choice):
    if server_choice == "1":
        return 12001
    elif server_choice == "2":
        return 12002
    elif server_choice == "3":
        return 12003
    else:
        print("Invalid server choice.")
        exit(1)

server_choice = input("Choose server ('(1) solve', '(2) available_jobs', or '(3) get_solution'): ")
serverPort = choose_server_port(server_choice)

clientSocket = socket(AF_INET, SOCK_STREAM)

clientSocket.connect(('127.0.0.1', serverPort))

message = input('Input number of robots: ')
clientSocket.send(message.encode())

modMessage = clientSocket.recv(2048).decode()
print('From server:', modMessage)

clientSocket.close()