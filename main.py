import data_read

depot_data, jobs_data, travel_data, general_data, vehicle_data = data_read.get_data(1, 10, 3, 1)

# reus is people, non reus is equipment

periods = general_data[0]
jobs = general_data[1]  # also num of turbines
vehicles = general_data[2]
people_types = general_data[3]

# Create the nodes size = 2n+2 with final two nodes as depot pickup and delivery, n = num jobs
N = jobs + 2

routes = []