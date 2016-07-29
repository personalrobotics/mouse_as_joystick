import os, subprocess



def disable_mouse_by_id(id):
    subprocess.call('xinput -disable ' + str(id), shell=True)

def get_mouse_filename_from_id(id):
    print id
    event_address_all = subprocess.check_output("xinput list-props " + str(id) + " | grep -Eo '/dev/input/.*[0-9]{1,2}'", shell=True).rstrip()
    event_address = event_address_all.split('/')[-1] # get last item

    # look at /proc/bus/input/devices to see which mouse this event maps to
    mouse_address = subprocess.check_output("cat /proc/bus/input/devices | grep -Eo 'mouse[0-9]{1,2} " + event_address + "'", shell=True).split()[0]
    return '/dev/input/'+mouse_address


def make_mouse_readable(mouse_filename):
    #first check if file is not readable
    if not os.access(mouse_filename, os.R_OK):
      #if not readable, change permissions
      #this line might potentially ask for a password
      subprocess.call('sudo chmod a+r ' + mouse_filename, shell=True)

def get_mouse_process_id():
    xinput_id_output = subprocess.check_output("xinput list | grep -Eo 'ouse.*id\=[0-9]{1,2}' | grep -Eo '[0-9]{1,2}'", shell=True)
    xinput_id_split = xinput_id_output.split("\n")

    #assume it is the last device id
    for candidate_id in reversed(xinput_id_split):
        if len(candidate_id) > 0:
            return int(candidate_id)

    return None

