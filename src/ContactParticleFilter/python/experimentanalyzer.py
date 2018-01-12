import os
import time
import tinydb
import numpy as np
import math

from director import ioUtils

import spartan.utils.utils as spartanUtils

import utils as cpfUtils
import cpf_lcmtypes

import lcm


class MessageContainer(object):

    def __init__(self, messageClass=None):
        self.timestamps = np.array([])
        self.messages = []
        self.messageClass = messageClass

    def insert_message(self, timestamp, msg):
        self.timestamps = np.append(self.timestamps, timestamp)
        self.messages.append(msg)

    def insert_message_data(self, timestamp, data):
        msg = self.messageClass.decode(data)
        self.insert_message(timestamp, msg)

    def get_message(self, timestamp):
        idx = self.timestamps.searchsorted(timestamp)
        idx = min(idx, len(self.messages)-1) # don't index out of range
        return self.messages[idx]

class ExperimentAnalyzer(object):

    """
    Mode can be either "simulation" or "hardware"
    """
    def __init__(self, logFolderName="20180105-165005_simulation", mode="simulation"):
        self.logFolderName = logFolderName

        self.cpfSourceDir = cpfUtils.getCPFSourceDir()
        self.logFolder = os.path.join(cpfUtils.getCPFDataDir(), logFolderName)
        self.loadDatabase()
        self.mode = mode

        self.initialize()

    def initialize(self):
        self.channels_info = dict()
        self.channels_info["EXTERNAL_CONTACT_LOCATION"] = cpf_lcmtypes.multiple_contact_location_t
        self.channels_info["CONTACT_FILTER_POINT_ESTIMATE"] = cpf_lcmtypes.contact_filter_estimate_t
        self.channels_info["FORCE_PROBE_DATA"] = cpf_lcmtypes.single_contact_filter_estimate_t

        # create a new database
        # remove old database file
        db_analysis_file = os.path.join(self.logFolder, "db_analysis.json")
        
        # remove analysis file if it exists
        try:
            os.remove(db_analysis_file)
        except OSError:
            pass
        

        self.db_analysis = tinydb.TinyDB(db_analysis_file)

    def loadDatabase(self):
        db_file = os.path.join(self.logFolder, "db.json")
        self.db = tinydb.TinyDB(db_file)


    """
    Compute some statistics from the log
    @:arg db_entry - the database entry for this log
    """
    def analyzeLog(self, lcm_log_file):
        self.events = []
        lcm_log_reader = lcm.EventLog(lcm_log_file, mode='r')
        self.lcm_log_reader = lcm_log_reader


        message_containers = dict()

        for channel, messageClass in self.channels_info.iteritems():
            message_containers[channel] = MessageContainer(messageClass=messageClass)

        while True:
            event = self.lcm_log_reader.read_next_event()
            if event is None:
                break

            self.events.append(event)
            if event.channel in self.channels_info:
                message_containers[event.channel].insert_message_data(event.timestamp, event.data)


        # # store the results in a data structure
        # if self.mode == "simulation":
        #     stats = self.computeStatisticsForSim(message_containers)

        # if self.mode == "hardware":
        #     stats = self.computeStatisticsForHardware(message_containers)
        
        stats = self.computeStatistics(message_containers, mode=self.mode)
        return stats


    """
    Compute some statistics like average location error, average force error, etc.
    """
    def computeStatistics(self, message_containers, mode="simulation"):
        ground_truth = None
        
        if mode == "simulation":
            ground_truth = message_containers["EXTERNAL_CONTACT_LOCATION"]

        if mode == "hardware":
            ground_truth = message_containers["FORCE_PROBE_DATA"]

        estimate = message_containers["CONTACT_FILTER_POINT_ESTIMATE"]

        # don't use member variables here . . .
        self.stats_list = []
        self.position_error = []
        self.force_error = []

        for idx, msg in enumerate(estimate.messages):
            timestamp = estimate.timestamps[idx]
            
            ground_truth_msg = ground_truth.get_message(timestamp)
            
            # this is needed because the sim outputsd data in a different format
            if self.mode == "simulation":
                ground_truth_msg = ground_truth_msg.contacts[0]

            stats = self.computeSingleMessageStatistics(msg, ground_truth_msg, mode=self.mode)
            self.stats_list.append(stats)
            self.position_error.append(stats['contact_position_in_world'])
            self.force_error.append(stats['contact_force_in_world'])


        self.position_error = np.array(self.position_error)
        self.force_error = np.array(self.force_error)

        d = dict()
        d['position'] = dict()
        d['position']['mean'] = np.average(self.position_error)
        d['position']['std_dev'] = np.std(self.position_error)

        d['force'] = dict()
        d['force']['mean'] = np.average(self.force_error)
        d['force']['std_dev'] = np.std(self.force_error)

        self.stats = d
        return d


    """
    @:param msg: single_contact_filter_estimate_t
    Converts it to a dict of numpy arrays
    """
    def convertSingleContactFilterEstimateMessageToNumpyArray(self, msg):
        d = dict()
        d['contact_position_in_world'] = np.array(msg.contact_position_in_world)
        d['contact_force_in_world'] = np.array(msg.contact_force_in_world)
        return d

    """
    @:param estimate: contact_filter_estimate_t
    @:param actual: multiple_contact_location_t (on EXTERNAL_CONTACT_LOCATION channel)
    
    we will assume there is only a single contact point for simplicity, otherwise data associate becomes hard as well
    """
    def computeSingleMessageStatistics(self, estimate, actual, mode="simulation"):
        # do everything in world frame
        estimate_s = estimate.single_contact_estimate[0]
        actual_s = actual
        # estimate_s = estimate
        # actual_s = actual.contacts[0]


        data_actual = self.convertSingleContactFilterEstimateMessageToNumpyArray(actual_s)
        data_est = self.convertSingleContactFilterEstimateMessageToNumpyArray(estimate_s)


        stats_names = ["contact_position_in_world"]
        stats = dict()
        for stat_name in stats_names:
            delta = data_actual[stat_name] - data_est[stat_name]
            l2_norm = np.linalg.norm(delta)
            stats[stat_name] = l2_norm

        # custom scaling logic for contact force in world
        if mode == "simulation":
            name = "contact_force_in_world"
            delta = (data_actual[name] - data_est[name]) / np.linalg.norm(data_actual[name])
            l2_norm = np.linalg.norm(delta)
            stats[name] = l2_norm

        if mode == "hardware":
            name = "contact_force_in_world"
            force_direction_actual = data_actual[name] / np.linalg.norm(data_actual[name])
            force_direction_est =  data_est[name] / np.linalg.norm(data_est[name])

            delta = force_direction_actual - force_direction_est
            l2_norm = np.linalg.norm(delta)
            stats[name] = l2_norm


        # this is sort of meaningless for hardware mode
        name = "contact_force_in_world"
        stats["contact_force_magnitude"] = abs( (np.linalg.norm(data_actual[name]) - np.linalg.norm(data_est[name])) / np.linalg.norm(data_actual[name]) )

        return stats

    """
    Analyze all the logs in the database    
    """
    def run(self):
        db_entries = self.db.all()

        # run analysis for each db entry, create new field (called stats) then store it
        for entry in db_entries:
            lcm_log_file = os.path.join(self.logFolder, entry['lcm_log_file'])
            stats = self.analyzeLog(lcm_log_file)
            entry['stats'] = stats
            self.db_analysis.insert(entry)

    def test(self):
        query = tinydb.Query()
        force_name = "iiwa_link_5_1"
        pose_name = "q_nom"

        db_entries = self.db.search((query.force_name == force_name) & (query.pose_name == pose_name) )

        self.db_entries = db_entries

        self.db_entry = db_entries[0]
        # self.lcm_log_file = self.db_entry["lcm_log_name"]
