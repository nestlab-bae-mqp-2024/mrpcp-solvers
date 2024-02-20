from flask import Flask, request, jsonify, make_response

app = Flask(__name__)

@app.route("/solve", methods=['POST'])
def hello_world():
    k = request.args.get('k')
    q_k = request.args.get('q_k')
    n_a = request.args.get('n_a')
    rp = request.args.get('rp')
    l = request.args.get('l')
    d = request.args.get('d')
    mode = request.args.get('mode')

    response = {
        "params": {
            "k": k,
            "q_k": q_k,
            "n_a": n_a,
            "rp": rp,
            "l": l,
            "d": d,
            "mode": mode,
        },
        "robot_world_path": [
            [[0., 0.], [0.5, 0.5]]
        ],
        "robot_node_path": [[1, 0]],
        "job_id": "",
        "status": "complete"
    }
    return make_response(
        jsonify(response),
        200)


app.run(host="127.0.0.1")

