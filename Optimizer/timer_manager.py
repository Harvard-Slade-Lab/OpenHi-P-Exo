from PyQt5 import QtCore
from collections import defaultdict

class TimerManager:
    """
    Singleton class to manage all timers in the application.
    """
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(TimerManager, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if not self._initialized:
            self._timers = {}  
            self._callbacks = defaultdict(list) 
            self._initialized = True
    
    def register_callback(self, callback, interval=100):
        """
        Register a callback to be executed at the specified interval.
        
        Args:
            callback: Function to call when timer fires
            interval: Timer interval in milliseconds
        """
        # Create timer if it doesn't exist for this interval
        if interval not in self._timers:
            timer = QtCore.QTimer()
            timer.timeout.connect(lambda: self._dispatch_callbacks(interval))
            timer.start(interval)
            self._timers[interval] = timer
        
        # Register callback
        self._callbacks[interval].append(callback)
        return True
    
    def _dispatch_callbacks(self, interval):
        """Call all callbacks registered for the given interval."""
        for callback in self._callbacks[interval]:
            try:
                callback()
            except Exception as e:
                print(f"Error in timer callback: {e}")
    
    def unregister_callback(self, callback, interval=None):
        """
        Unregister a callback.
        
        Args:
            callback: Function to unregister
            interval: Timer interval (if None, search all intervals)
        """
        if interval is not None and interval in self._callbacks:
            if callback in self._callbacks[interval]:
                self._callbacks[interval].remove(callback)
                return True
        else:
            # Search all intervals
            for i, callbacks in self._callbacks.items():
                if callback in callbacks:
                    callbacks.remove(callback)
                    return True
        return False