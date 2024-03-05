import sys
import os

# gets location of getCameraFeeds to be recognized as a module
dir = os.path.dirname(__file__)
filename = os.path.join(dir, '../src')
sys.path.insert(0,filename)

import getCameraFeeds as gcf

# Runnable to display camera feeds
camHandler = gcf.CameraHandler()
camHandler.run_feeds()
