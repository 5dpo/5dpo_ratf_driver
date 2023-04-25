#!/usr/bin/env python

import rosbag
import csv

bag_file = "../bag/log_tune_batt-11.73_2023-04-25-19-03-44.bag"
output_folder = "../bag/"

# Open the bag file
bag = rosbag.Bag(bag_file)

# Get the list of topics in the bag file
topics = bag.get_type_and_topic_info()[1].keys()

# Loop through each topic and convert it to CSV
for topic in topics:
    # Create the output CSV file
    output_file = output_folder + topic.replace('/', '_') + ".csv"
    with open(output_file, 'w') as csvfile:
        writer = csv.writer(csvfile)

        # Write the header row
        writer.writerow(['Time', 'Data'])

        # Loop through each message in the topic and write it to the CSV file
        for _, msg, t in bag.read_messages(topics=topic):
            writer.writerow([t.to_sec(), msg])

# Close the bag file
bag.close()
