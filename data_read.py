import os
import pathlib
import numpy as np

BASE_PATH = str(pathlib.Path().resolve()) # get current path
FILE_NAME = "{}n{}T{}"  # % will change based on inputs
DEPOT_FILE = "depotinfo_inst{}"
GENERAL_FILE = "genInfo_inst{}"
JOBS_FILE = "jobs_inst{}"
TRAVEL_FILE = "maxtravel_inst{}"


def load_data(path: str, instance: str, periods):
	jobs_cols = (i for i in range(8+periods))

	depots_path = path + "/" + DEPOT_FILE.format(instance)
	jobs_path = path + "/" + JOBS_FILE.format(instance)
	travel_path = path + "/" + TRAVEL_FILE.format(instance)
	general_path = path + "/" + GENERAL_FILE.format(instance)


	# General file contains general and vehicle info
	general_data = np.loadtxt(general_path, delimiter=' ', skiprows=1, max_rows=1)
	vehicle_data = np.loadtxt(general_path, delimiter=' ', skiprows=3)

	travel_cols = (i for i in range(len(vehicle_data)))


	# One type of data per file
	depot_data = np.loadtxt(depots_path, delimiter=' ', skiprows=1)
	jobs_data = np.loadtxt(jobs_path, delimiter=' ', skiprows=1, usecols=jobs_cols)
	travel_data = np.loadtxt(travel_path, delimiter=' ', usecols=travel_cols)

	return depot_data, jobs_data, travel_data, general_data, vehicle_data


def get_data(folder, jobs, periods, instance):
	file = FILE_NAME.format(str(folder), str(jobs), str(periods))

	path = BASE_PATH + "/data/" + str(folder) + "/" + file
	return load_data(path, str(instance), periods)


def get_data_with_inputs():
	folder = input("Select folder (1-3): ")
	jobs = input("Job Count: ")
	periods = input("Turbine Count: ")
	instance = input("Instance (1-3): ")

	file = FILE_NAME.format(folder, jobs, periods)

	path = BASE_PATH + "/data/" + folder + "/" + file
	return load_data(path, instance, periods)

