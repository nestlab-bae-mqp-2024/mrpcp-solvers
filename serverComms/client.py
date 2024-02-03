import requests

def get_user_input():
    param1 = input("Enter number of robots: ")
    param2 = input("Enter q_k parameter: ")
    param3 = input("Enter number of nodes per axis: ")
    return param1, param2, param3

def send_request_to_server(endpoint_url, param1, param2, param3):
    data = {'k': param1, 'q_k': param2, 'n': param3}
    response = requests.post(endpoint_url, json=data)

    if response.status_code == 200:
        print('Request sent successfully.')
        print('Response content:', response.content.decode('utf-8'))
    else:
        print(f'Request failed with status code: {response.status_code}')

if __name__ == '__main__':
    server_url = 'http://127.0.0.1:5000/solve'

    # Get user input for parameters
    param1_to_send, param2_to_send, param3_to_send = get_user_input()

    # Send the request to the server
    send_request_to_server(server_url, param1_to_send, param2_to_send, param3_to_send)
