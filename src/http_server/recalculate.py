"""
Author: Samara Holmes
Date: 2/17/2024

This program defines the methods relating to recalculating the paths for robots after a failure occurs
"""
import math
import os
from flask import json

from src.http_server.heuristic2 import *
from src.http_server.json_handlers import saveResultsToCache
from src.http_server.mrpcp import saveGraphPath, convertToWorldPath

def convertToNodePath(world_path):
    node_path = []
    for i in range(len(world_path)):
        if i == 0:
            node_path.append([world_path[i]])
        else:
            node_path.append([world_path[i], world_path[i - 1]])
    return node_path


def getParamsFromJobId(job_id):
    """
    Function to return all the MRPCP and Heuristic parameters based on the job id
    It takes the job id and splits the id into the parameters
    :param job_id:
    :return:
    """
    k, q_k, n_a, rp, l, d, mode = job_id.split('_')
    return k, q_k, n_a, rp, l, d, mode
