#!/usr/bin/env python3

# Copyright (c) 2022 Lake Fusion Technologies GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
from datetime import datetime as dt


class RecordingCamera(object):
    """
    A simple camera class for storing frames
    """
    _frameNr = 0

    def __init__(self, actor, recordingPath, camId, dataIndex):
        try:
            camStr = "cam" + str(camId)
            os.makedirs(os.path.join(recordingPath, camStr))
            self._recording = True
            self._filenamePrefix = os.path.join(recordingPath, camStr, camStr + "_")

        except OSError as error:
            print(error)
            # do not record if the dir creation fails
            self._recording = False

        self._dataIndex = dataIndex

    def save(self, ts, data):
        if self._recording:
            milliseconds = str(dt.fromtimestamp(ts).time().microsecond * 1000).zfill(9)
            dtStr = dt.fromtimestamp(ts).strftime("_%Y%m%d_%H%M%S_") + milliseconds
            data[self._dataIndex].save_to_disk(self._filenamePrefix + str(self._frameNr).zfill(6) + dtStr + '__.jpg')
            self._frameNr += 1
