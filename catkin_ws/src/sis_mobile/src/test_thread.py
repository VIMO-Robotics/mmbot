import threading

def printit():
  threading.Timer(0.1, printit).start()
  print "Hello, World!"

def printto():
  threading.Timer(0.1, printit).start()
  print("You")

printit()
printto()