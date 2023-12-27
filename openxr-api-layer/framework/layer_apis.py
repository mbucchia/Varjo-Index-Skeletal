# The list of OpenXR functions our layer will override.
override_functions = [
    "xrGetSystem",
    "xrGetSystemProperties",
    "xrSuggestInteractionProfileBindings",
    "xrCreateSession",
    "xrDestroySession",
    "xrCreateHandTrackerEXT",
    "xrDestroyHandTrackerEXT",
    "xrLocateHandJointsEXT"
]

# The list of OpenXR functions our layer will use from the runtime.
# Might repeat entries from override_functions above.
requested_functions = [
    "xrGetInstanceProperties",
    "xrGetSystemProperties",
    "xrGetCurrentInteractionProfile",
    "xrPathToString",
    "xrStringToPath",
    "xrCreateActionSpace",
    "xrDestroySpace",
    "xrLocateSpace",
]

# The list of OpenXR extensions our layer will either override or use.
extensions = ['XR_EXT_hand_tracking']
