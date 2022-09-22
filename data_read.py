import os
import pathlib
import numpy as np

BASE_PATH = str(pathlib.Path().resolve()) # get current path
FILE_NAME = "{}n{}T{}"  # % will change based on inputs
DEPOT_FILE = "depotinfo_inst{}"
GENERAL_FILE = "genInfo_inst{}"
JOBS_FILE = "jobs_inst{}"
TRAVEL_FILE = "maxtravel_inst{}"
JOBS_COLS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10)  # jobs file has blank space at end, ruins np.loadtxt


def load_data(path: str, instance: str):
	depots_path = path + "/" + DEPOT_FILE.format(instance)
	jobs_path = path + "/" + JOBS_FILE.format(instance)
	travel_path = path + "/" + TRAVEL_FILE.format(instance)
	general_path = path + "/" + GENERAL_FILE.format(instance)

	# One type of data per file
	depot_data = np.loadtxt(depots_path, delimiter=' ', skiprows=1)
	jobs_data = np.loadtxt(jobs_path, delimiter=' ', skiprows=1, usecols=JOBS_COLS)
	travel_data = np.loadtxt(travel_path, delimiter=' ', skiprows=1)

	# General file contains general and vehicle info
	general_data = np.loadtxt(general_path, delimiter=' ', skiprows=1, max_rows=1)
	vehicle_data = np.loadtxt(general_path, delimiter=' ', skiprows=3)

	return depot_data, jobs_data, travel_data, general_data, vehicle_data


def get_data(periods, jobs, turbines, instance):
	file = FILE_NAME.format(str(periods), str(jobs), str(turbines))

	path = BASE_PATH + "/data/" + str(periods) + "/" + file
	return load_data(path, str(instance))


def get_data_with_inputs():
	periods = input("Select periods (1-3): ")
	jobs = input("Job Count: ")
	turbines = input("Turbine Count: ")
	instance = input("Instance (1-3): ")

	file = FILE_NAME.format(periods, jobs, turbines)

	path = BASE_PATH + "/data/" + periods + "/" + file
	return load_data(path, instance)

