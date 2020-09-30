#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Benjamin Kelenyi"
__copyright__ = "Copyright 2020"
__email__ = "benjamin.kelenyi@gmail.com"
__doc__ = "This script is used to download a package from a specific location," \
          "and put to a specific destination. If the package already exist in " \
          "the destination folder, this will be overwritten."

import urllib.request
import argparse
import pysftp
import sys

""" Settings section for sFTP server """
myHostname = "rocon.utcluj.ro"
myUsername = "benji"
myPassword = "utcn-benji"


def parse_arg():
    """
    Function used to parce the arguments for the scrip
    Usage: test.py [-h] --source SOURCE --destination DESTINATION
    :return: args.source, args.destination
    """
    # Initiate the parser
    parser = argparse.ArgumentParser()

    # source from the package
    parser.add_argument("--source", "-s", help="source", required=True)

    # destination from the package
    parser.add_argument("--destination", "-d", help="destination", required=True)

    # Read arguments from the command line
    args = parser.parse_args()

    # check for arguments
    if args.source and args.destination:
        print("Source = %s" % args.source)
        print("Destination = %s" % args.destination)

    return args.source, args.destination


def download(url, destination):
    """
    This function is used to download a package from a specific URL
    :param url: source of the data
    :param destination: destination for the data
    :return: no return
    """

    with pysftp.Connection(host=myHostname, username=myUsername, password=myPassword) as sftp:
        print('Connection successfully stabilised ...')

        with pysftp.cd(str(destination)):
            print('Download in progress...')
            sftp.get(str(url))

    print("Package downloaded successfully to: ", args.destination)


if __name__ == '__main__':
    url, destination = parse_arg()
    download(url, destination)
