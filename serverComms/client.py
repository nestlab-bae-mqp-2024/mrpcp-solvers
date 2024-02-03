import requests

def get_user_input():
    endpoint = input("Enter endpoint (solve/available_jobs/get_solution): ")

    if endpoint == 'solve':
        param1 = input("Enter number of robots: ")
        param2 = input("Enter q_k parameter: ")
        param3 = input("Enter number of nodes per axis: ")
        return endpoint, param1, param2, param3
    elif endpoint == 'available_jobs' or endpoint == 'get_solution':
        return endpoint, None, None, None
    else:
        print('Invalid endpoint. Please choose from solve, available_jobs, or get_solution.')
        return None, None, None, None

def send_request_to_server(endpoint_url, param1, param2, param3):
    if endpoint_url.endswith('/solve'):
        data = {'k': param1, 'q_k': param2, 'n': param3}
        response = requests.post(endpoint_url, json=data)
    elif endpoint_url.endswith('/available_jobs') or endpoint_url.endswith('/get_solution'):
        response = requests.get(endpoint_url)
    else:
        print('Invalid endpoint. Exiting...')
        return None

    if response.status_code == 200:
        print('Request sent successfully.')
        print('Response content:', response.content.decode('utf-8'))
        return response
    else:
        print(f'Request failed with status code: {response.status_code}')
        return None

def get_available_jobs(endpoint_url):
    response = requests.get(endpoint_url)

    if response.status_code == 200:
        content = response.json()
        available_jobs = content.get('jobs', [])
        print('Available jobs:', available_jobs)
    else:
        print(f'Fetching available jobs failed with status code: {response.status_code}')

if __name__ == '__main__':
    server_base_url = 'http://127.0.0.1:5000/'

    # Get user input for endpoint and parameters
    endpoint_to_send, param1_to_send, param2_to_send, param3_to_send = get_user_input()

    if endpoint_to_send == 'solve':
        endpoint_url = server_base_url + endpoint_to_send
        send_request_to_server(endpoint_url, param1_to_send, param2_to_send, param3_to_send)
    elif endpoint_to_send == 'available_jobs':
        endpoint_url = server_base_url + endpoint_to_send
        get_available_jobs(endpoint_url)
    elif endpoint_to_send == 'get_solution':
        print('Fetching solution...')
    else:
        print('Invalid endpoint. Exiting...')
