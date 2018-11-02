#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2018 VECTIONEER.
#

import unittest
import motorcortex
import time

SERVER = 'localhost'

api_path = '../src/motorcortex_api/motorcortex-msg'


class MessageTypesTest(unittest.TestCase):

    def test(self):
        motorcortex_types = motorcortex.MessageTypes()
        motorcortex_msg, motionsl_msg = motorcortex_types.load(
            [{'proto': api_path + '/motorcortex_pb2.py', 'hash': api_path + '/motorcortex_hash.json'},
             {'proto': api_path + '/motionSL_pb2.py', 'hash': api_path + '/motionSL_hash.json'}])

        # check if namespaces exist
        self.assertTrue(motorcortex_types.getNamespace("motorcortex"))
        self.assertTrue(motorcortex_types.getNamespace("motion_spec"))

        # check if enums are loaded correctly
        self.assertTrue(hasattr(motorcortex_msg, 'OK'))
        # check if types are loaded
        self.assertTrue(hasattr(motorcortex_msg, 'ParameterMsg'))


class OpenRequestConnection(unittest.TestCase):

    def test(self):
        # Open request connection
        motorcortex_types = motorcortex.MessageTypes()
        parameter_tree = motorcortex.ParameterTree()
        req = motorcortex.Request(motorcortex_types, parameter_tree)
        if req.connect("ws://%s:5558" % SERVER):
            print("Request connection is etablished")
        else:
            print("Failed to establish Request connection")
            self.assertTrue(False)

        req.close()


class OpenSubscribeConnection(unittest.TestCase):

    def test(self):
        # Open subscribe connection
        motorcortex_types = motorcortex.MessageTypes()
        parameter_tree = motorcortex.ParameterTree()
        req = motorcortex.Request(motorcortex_types, parameter_tree)
        req.connect("ws://%s:5558" % SERVER)
        sub = motorcortex.Subscribe(req, motorcortex_types)
        if sub.connect("ws://%s:5557" % SERVER):
            print("Subscribe connection is etablished")
        else:
            print("Failed to establish Subscribe connection")
            self.assertTrue(False)

        sub.close()
        req.close()


class Login(unittest.TestCase):

    def test(self):
        # Open request connection
        motorcortex_types = motorcortex.MessageTypes()
        motorcortex_msg, motionsl_msg = motorcortex_types.load(
            [{'proto': api_path + '/motorcortex_pb2.py', 'hash': api_path + '/motorcortex_hash.json'},
             {'proto': api_path + '/motionSL_pb2.py', 'hash': api_path + '/motionSL_hash.json'}])

        parameter_tree = motorcortex.ParameterTree()
        req = motorcortex.Request(motorcortex_types, parameter_tree)
        if req.connect("ws://%s:5558" % SERVER):
            print("Request connection is etablished")
        else:
            print("Failed to establish Request connection")
            self.assertTrue(False)

        reply = req.login("operator", "operat")
        result = reply.get()
        if result.status == motorcortex_msg.OK:
            print("Login successfull")
        else:
            print("Failed to login")
            self.assertTrue(False)

        req.close()


class DecodeEncode(unittest.TestCase):

    def test(self):
        motorcortex_types = motorcortex.MessageTypes()
        motorcortex_msg, motionsl_msg = motorcortex_types.load(
            [{'proto': api_path + '/motorcortex_pb2.py', 'hash': api_path + '/motorcortex_hash.json'},
             {'proto': api_path + '/motionSL_pb2.py', 'hash': api_path + '/motionSL_hash.json'}])

        # check if namespaces exist
        self.assertTrue(motorcortex_types.getNamespace("motorcortex"))
        self.assertTrue(motorcortex_types.getNamespace("motion_spec"))

        # check if enums are loaded correctly
        self.assertTrue(hasattr(motorcortex_msg, 'OK'))
        # check if types are loaded
        self.assertTrue(hasattr(motorcortex_msg, 'ParameterMsg'))

        login_msg = motorcortex_types.createType('motorcortex.LoginMsg')
        login_msg.login = 'operator'
        login_msg.password = 'operat'
        res = login_msg.SerializeToString()

        login_msg1 = motorcortex_types.createType('motorcortex.LoginMsg')
        login_msg1.ParseFromString(res)

        self.assertTrue(login_msg.login == login_msg1.login)
        self.assertTrue(login_msg.password == login_msg1.password)

        # param_tree_msg = motorcortex_types.createType('motorcortex.ParameterTreeMsg')
        # param_tree_msg.hash = 01245
        # param_tree_msg.status = 1
        # res = param_tree_msg.SerializeToString()
        #
        # param_tree_msg1 = motorcortex_types.createType('motorcortex.ParameterTreeMsg')
        # param_tree_msg1.ParseFromString(res)
        #
        # self.assertTrue(param_tree_msg.hash == param_tree_msg1.hash)
        # self.assertTrue(param_tree_msg.status == param_tree_msg1.status)


class ParameterTree(unittest.TestCase):

    def test(self):
        # Open request connection
        motorcortex_types = motorcortex.MessageTypes()
        motorcortex_msg, motionsl_msg = motorcortex_types.load(
            [{'proto': api_path + '/motorcortex_pb2.py', 'hash': api_path + '/motorcortex_hash.json'},
             {'proto': api_path + '/motionSL_pb2.py', 'hash': api_path + '/motionSL_hash.json'}])

        parameter_tree = motorcortex.ParameterTree()
        req = motorcortex.Request(motorcortex_types, parameter_tree)
        if req.connect("ws://%s:5558" % SERVER):
            print("Request connection is etablished")
        else:
            print("Failed to establish Request connection")
            self.assertTrue(False)

        reply = req.login("operator", "operat")
        result = reply.get()
        if result.status == motorcortex_msg.OK:
            print("Login successfull")
        else:
            print("Failed to login")
            self.assertTrue(False)

        # Requesting a parameter tree
        reply = req.getParameterTree()
        result = reply.get()
        parameter_tree.load(result)
        if result.status == motorcortex_msg.OK:
            print("Got parameter tree")
        else:
            print("Failed to get parameter tree")
            self.assertTrue(False)

        req.close()


class SetGetParameter(unittest.TestCase):

    def test(self):
        # Open request connection
        motorcortex_types = motorcortex.MessageTypes()
        motorcortex_msg, motionsl_msg = motorcortex_types.load(
            [{'proto': api_path + '/motorcortex_pb2.py', 'hash': api_path + '/motorcortex_hash.json'},
             {'proto': api_path + '/motionSL_pb2.py', 'hash': api_path + '/motionSL_hash.json'}])

        parameter_tree = motorcortex.ParameterTree()
        req = motorcortex.Request(motorcortex_types, parameter_tree)
        if req.connect("ws://%s:5558" % SERVER):
            print("Request connection is etablished")
        else:
            print("Failed to establish Request connection")
            self.assertTrue(False)

        reply = req.login("operator", "operat")
        result = reply.get()
        if result.status == motorcortex_msg.OK:
            print("Login successfull")
        else:
            print("Failed to login")
            self.assertTrue(False)

        # Requesting a parameter tree
        reply = req.getParameterTree()
        result = reply.get()
        parameter_tree.load(result)
        if result.status == motorcortex_msg.OK:
            print("Got parameter tree")
        else:
            print("Failed to get parameter tree")
            self.assertTrue(False)

        reply = req.getParameter('root/Control/dummy')
        msg = reply.get()
        value = msg.value[0]

        value = not value
        reply = req.setParameter('root/Control/dummy', value)
        reply.get();

        reply = req.getParameter('root/Control/dummy')
        msg = reply.get()
        new_value = msg.value[0]

        self.assertTrue(value == new_value)

        req.close()


class SubParameter(unittest.TestCase):
    counter = 0

    def newValue(self, value):
        self.counter = self.counter + 1

    def test(self):
        # Open request connection
        motorcortex_types = motorcortex.MessageTypes()
        motorcortex_msg, motionsl_msg = motorcortex_types.load(
            [{'proto': api_path + '/motorcortex_pb2.py', 'hash': api_path + '/motorcortex_hash.json'},
             {'proto': api_path + '/motionSL_pb2.py', 'hash': api_path + '/motionSL_hash.json'}])

        parameter_tree = motorcortex.ParameterTree()
        req = motorcortex.Request(motorcortex_types, parameter_tree)
        if req.connect("ws://%s:5558" % SERVER):
            print("Request connection is etablished")
        else:
            print("Failed to establish Request connection")
            self.assertTrue(False)

        reply = req.login("operator", "operat")
        result = reply.get()
        if result.status == motorcortex_msg.OK:
            print("Login successfull")
        else:
            print("Failed to login")
            self.assertTrue(False)

        # Requesting a parameter tree
        reply = req.getParameterTree()
        result = reply.get()
        parameter_tree.load(result)
        if result.status == motorcortex_msg.OK:
            print("Got parameter tree")
        else:
            print("Failed to get parameter tree")
            self.assertTrue(False)

        sub = motorcortex.Subscribe(req, motorcortex_types)
        if sub.connect("ws://%s:5557" % SERVER):
            print("Subscribe connection is etablished")
        else:
            print("Failed to establish Subscribe connection")
            self.assertTrue(False)

        subscription = sub.subscribe(['root/Control/dummy'], 'group1', 100)
        self.assertTrue(subscription.get())

        subscription.notify(self.newValue)
        timestamp = 0
        for x in range(4):
            timestamp = subscription.read()[0].timestamp
            print(timestamp)
            time.sleep(1)

        sub.unsubscribe(subscription)

        self.assertTrue(self.counter > 0)
        self.assertTrue(timestamp.sec > 0)

        sub.close()
        req.close()


if __name__ == '__main__':
    unittest.main()
