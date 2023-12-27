// MIT License
//
// Copyright(c) 2022-2023 Matthieu Bucchianeri
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this softwareand associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright noticeand this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "pch.h"

#include "layer.h"
#include <log.h>
#include <util.h>

namespace {
    // From https://github.com/ValveSoftware/openvr/wiki/Hand-Skeleton
    typedef int32_t BoneIndex_t;
    const BoneIndex_t INVALID_BONEINDEX = -1;
    enum HandSkeletonBone : BoneIndex_t {
        eBone_Root = 0,
        eBone_Wrist,
        eBone_Thumb0,
        eBone_Thumb1,
        eBone_Thumb2,
        eBone_Thumb3,
        eBone_IndexFinger0,
        eBone_IndexFinger1,
        eBone_IndexFinger2,
        eBone_IndexFinger3,
        eBone_IndexFinger4,
        eBone_MiddleFinger0,
        eBone_MiddleFinger1,
        eBone_MiddleFinger2,
        eBone_MiddleFinger3,
        eBone_MiddleFinger4,
        eBone_RingFinger0,
        eBone_RingFinger1,
        eBone_RingFinger2,
        eBone_RingFinger3,
        eBone_RingFinger4,
        eBone_PinkyFinger0,
        eBone_PinkyFinger1,
        eBone_PinkyFinger2,
        eBone_PinkyFinger3,
        eBone_PinkyFinger4,
        eBone_Aux_Thumb,
        eBone_Aux_IndexFinger,
        eBone_Aux_MiddleFinger,
        eBone_Aux_RingFinger,
        eBone_Aux_PinkyFinger,
        eBone_Count
    };

    inline float DegreeToRad(float deg) {
#define MATH_DOUBLE_PI 3.14159265358979323846
#define MATH_DOUBLE_TWOPI (2 * MATH_DOUBLE_PI)
#define MATH_DOUBLE_DEGREETORADFACTOR (MATH_DOUBLE_TWOPI / 360.0)
        return (float)(deg * MATH_DOUBLE_DEGREETORADFACTOR);
    }
} // namespace

namespace openxr_api_layer {

    using namespace log;
    using namespace xr::math;
    using namespace vr;

    // Our API layer implement these extensions, and their specified version.
    const std::vector<std::pair<std::string, uint32_t>> advertisedExtensions = {
        {XR_EXT_HAND_TRACKING_EXTENSION_NAME, 4}};

    // Initialize these vectors with arrays of extensions to block and implicitly request for the instance.
    //
    // Note that we block and implicitly request XR_EXT_hand_tracking in order to allow passthrough of it to the
    // runtime, in case we detect after instance creation that the upstream API layers or runtime are adequate.
    const std::vector<std::string> blockedExtensions = {XR_EXT_HAND_TRACKING_EXTENSION_NAME};
    const std::vector<std::string> implicitExtensions = {XR_EXT_HAND_TRACKING_EXTENSION_NAME};

    // This class implements our API layer.
    class OpenXrLayer : public openxr_api_layer::OpenXrApi {
      public:
        OpenXrLayer() = default;
        ~OpenXrLayer() = default;

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrGetInstanceProcAddr
        XrResult xrGetInstanceProcAddr(XrInstance instance, const char* name, PFN_xrVoidFunction* function) override {
            TraceLoggingWrite(g_traceProvider,
                              "xrGetInstanceProcAddr",
                              TLXArg(instance, "Instance"),
                              TLArg(name, "Name"),
                              TLArg(m_bypassApiLayer, "Bypass"));

            XrResult result = m_bypassApiLayer ? m_xrGetInstanceProcAddr(instance, name, function)
                                               : OpenXrApi::xrGetInstanceProcAddr(instance, name, function);

            TraceLoggingWrite(g_traceProvider, "xrGetInstanceProcAddr", TLPArg(*function, "Function"));

            return result;
        }

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrCreateInstance
        XrResult xrCreateInstance(const XrInstanceCreateInfo* createInfo) override {
            if (createInfo->type != XR_TYPE_INSTANCE_CREATE_INFO) {
                return XR_ERROR_VALIDATION_FAILURE;
            }

            // Needed to resolve the requested function pointers.
            OpenXrApi::xrCreateInstance(createInfo);

            // Dump the application name, OpenXR runtime information and other useful things for debugging.
            TraceLoggingWrite(g_traceProvider,
                              "xrCreateInstance",
                              TLArg(xr::ToString(createInfo->applicationInfo.apiVersion).c_str(), "ApiVersion"),
                              TLArg(createInfo->applicationInfo.applicationName, "ApplicationName"),
                              TLArg(createInfo->applicationInfo.applicationVersion, "ApplicationVersion"),
                              TLArg(createInfo->applicationInfo.engineName, "EngineName"),
                              TLArg(createInfo->applicationInfo.engineVersion, "EngineVersion"),
                              TLArg(createInfo->createFlags, "CreateFlags"));
            Log(fmt::format("Application: {}\n", createInfo->applicationInfo.applicationName));

            for (uint32_t i = 0; i < createInfo->enabledApiLayerCount; i++) {
                TraceLoggingWrite(
                    g_traceProvider, "xrCreateInstance", TLArg(createInfo->enabledApiLayerNames[i], "ApiLayerName"));
            }
            bool requestedHandTracking = false;
            for (uint32_t i = 0; i < createInfo->enabledExtensionCount; i++) {
                TraceLoggingWrite(
                    g_traceProvider, "xrCreateInstance", TLArg(createInfo->enabledExtensionNames[i], "ExtensionName"));

                const std::string_view ext(createInfo->enabledExtensionNames[i]);
                TraceLoggingWrite(g_traceProvider, "xrCreateInstance", TLArg(ext.data(), "ExtensionName"));
                if (ext == XR_EXT_HAND_TRACKING_EXTENSION_NAME) {
                    requestedHandTracking = true;
                }
            }

            XrInstanceProperties instanceProperties = {XR_TYPE_INSTANCE_PROPERTIES};
            CHECK_XRCMD(OpenXrApi::xrGetInstanceProperties(GetXrInstance(), &instanceProperties));
            const auto runtimeName = fmt::format("{} {}.{}.{}",
                                                 instanceProperties.runtimeName,
                                                 XR_VERSION_MAJOR(instanceProperties.runtimeVersion),
                                                 XR_VERSION_MINOR(instanceProperties.runtimeVersion),
                                                 XR_VERSION_PATCH(instanceProperties.runtimeVersion));
            TraceLoggingWrite(g_traceProvider, "xrCreateInstance", TLArg(runtimeName.c_str(), "RuntimeName"));
            Log(fmt::format("Using OpenXR runtime: {}\n", runtimeName));

            // Bypass the API layer unless the application requested the hand tracking extension and we are running on
            // the Varjo OpenXR runtime.
            Log(fmt::format("Application is{} requesting XR_EXT_hand_tracking\n", requestedHandTracking ? "" : " NOT"));
            m_bypassApiLayer = !requestedHandTracking || runtimeName.find("Varjo OpenXR Runtime") != 0;
            if (m_bypassApiLayer) {
                Log(fmt::format("{} layer will be bypassed\n", LayerName));
                return XR_SUCCESS;
            }

            return XR_SUCCESS;
        }

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrGetSystem
        XrResult xrGetSystem(XrInstance instance, const XrSystemGetInfo* getInfo, XrSystemId* systemId) override {
            if (getInfo->type != XR_TYPE_SYSTEM_GET_INFO) {
                return XR_ERROR_VALIDATION_FAILURE;
            }

            TraceLoggingWrite(g_traceProvider,
                              "xrGetSystem",
                              TLXArg(instance, "Instance"),
                              TLArg(xr::ToCString(getInfo->formFactor), "FormFactor"));

            const XrResult result = OpenXrApi::xrGetSystem(instance, getInfo, systemId);
            if (XR_SUCCEEDED(result) && getInfo->formFactor == XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY) {
                if (*systemId != m_systemId) {
                    // Check if the system supports hand tracking.
                    XrSystemHandTrackingPropertiesEXT handTrackingProperties{
                        XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT};
                    XrSystemProperties systemProperties{XR_TYPE_SYSTEM_PROPERTIES, &handTrackingProperties};
                    CHECK_XRCMD(OpenXrApi::xrGetSystemProperties(instance, *systemId, &systemProperties));
                    TraceLoggingWrite(g_traceProvider,
                                      "xrGetSystem",
                                      TLArg(systemProperties.systemName, "SystemName"),
                                      TLArg(!!handTrackingProperties.supportsHandTracking, "SupportsHandTracking"));
                    Log(fmt::format("Using OpenXR system: {}\n", systemProperties.systemName));

                    // TODO: Detect passthrough.

                    EVRInitError error{};
                    VR_Init(&error, VRApplication_Background, nullptr);
                    Log(fmt::format("VRInit returned: {}\n", error).c_str());
                    if (error == VRInitError_None) {
                        DetourMethodAttach(VRInput(), 0, hooked_SetActionManifestPath, original_SetActionManifestPath);

                        // Mark the XrSystemId to use.
                        m_systemId = *systemId;
                    }
                }
            }

            TraceLoggingWrite(g_traceProvider, "xrGetSystem", TLArg((int)*systemId, "SystemId"));

            return result;
        }

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrGetSystemProperties
        XrResult xrGetSystemProperties(XrInstance instance, XrSystemId systemId, XrSystemProperties* properties) {
            if (properties->type != XR_TYPE_SYSTEM_PROPERTIES) {
                return XR_ERROR_VALIDATION_FAILURE;
            }

            TraceLoggingWrite(g_traceProvider, "xrGetSystemProperties", TLArg(systemId, "SystemId"));

            const XrResult result = OpenXrApi::xrGetSystemProperties(instance, systemId, properties);
            if (XR_SUCCEEDED(result)) {
                if (isSystemHandled(systemId)) {
                    XrSystemHandTrackingPropertiesEXT* handTrackingProperties =
                        reinterpret_cast<XrSystemHandTrackingPropertiesEXT*>(properties->next);
                    while (handTrackingProperties) {
                        if (handTrackingProperties->type == XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT) {
                            break;
                        }
                        handTrackingProperties =
                            reinterpret_cast<XrSystemHandTrackingPropertiesEXT*>(handTrackingProperties->next);
                    }

                    if (handTrackingProperties) {
                        handTrackingProperties->supportsHandTracking = XR_TRUE;
                    }
                }
            }

            return result;
        }

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrCreateSession
        XrResult xrCreateSession(XrInstance instance,
                                 const XrSessionCreateInfo* createInfo,
                                 XrSession* session) override {
            if (createInfo->type != XR_TYPE_SESSION_CREATE_INFO) {
                return XR_ERROR_VALIDATION_FAILURE;
            }

            TraceLoggingWrite(g_traceProvider,
                              "xrCreateSession",
                              TLXArg(instance, "Instance"),
                              TLArg((int)createInfo->systemId, "SystemId"),
                              TLArg(createInfo->createFlags, "CreateFlags"));

            const XrResult result = OpenXrApi::xrCreateSession(instance, createInfo, session);
            if (XR_SUCCEEDED(result)) {
                if (isSystemHandled(createInfo->systemId)) {
                    CHECK_XRCMD(xrStringToPath(GetXrInstance(), "/user/hand/left", &m_subActionPath[xr::Side::Left]));
                    CHECK_XRCMD(xrStringToPath(GetXrInstance(), "/user/hand/right", &m_subActionPath[xr::Side::Right]));

                    m_manifestUpdated = false;
                    m_gripSpace[xr::Side::Left] = m_gripSpace[xr::Side::Right] = XR_NULL_HANDLE;
                    m_actionSet = k_ulInvalidActionSetHandle;
                    m_skeletonAction[xr::Side::Left] = m_skeletonAction[xr::Side::Right] = k_ulInvalidActionHandle;
                    m_session = *session;
                }

                TraceLoggingWrite(g_traceProvider, "xrCreateSession", TLXArg(*session, "Session"));
            }

            return result;
        }

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrDestroySession
        XrResult xrDestroySession(XrSession session) override {
            TraceLoggingWrite(g_traceProvider, "xrDestroySession", TLXArg(session, "Session"));

            const XrResult result = OpenXrApi::xrDestroySession(session);
            if (XR_SUCCEEDED(result)) {
                if (isSessionHandled(session)) {
                    if (m_gripSpace[xr::Side::Left] != XR_NULL_HANDLE) {
                        xrDestroySpace(m_gripSpace[xr::Side::Left]);
                        m_gripSpace[xr::Side::Left] = XR_NULL_HANDLE;
                    }
                    if (m_gripSpace[xr::Side::Right] != XR_NULL_HANDLE) {
                        xrDestroySpace(m_gripSpace[xr::Side::Right]);
                        m_gripSpace[xr::Side::Right] = XR_NULL_HANDLE;
                    }
                    m_actionSet = k_ulInvalidActionSetHandle;
                    m_skeletonAction[xr::Side::Left] = m_skeletonAction[xr::Side::Right] = k_ulInvalidActionHandle;

                    m_session = XR_NULL_HANDLE;
                }
            }

            return result;
        }

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrSuggestInteractionProfileBindings
        XrResult xrSuggestInteractionProfileBindings(
            XrInstance instance, const XrInteractionProfileSuggestedBinding* suggestedBindings) override {
            if (suggestedBindings->type != XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING) {
                return XR_ERROR_VALIDATION_FAILURE;
            }

            TraceLoggingWrite(g_traceProvider,
                              "xrSuggestInteractionProfileBindings",
                              TLXArg(instance, "Instance"),
                              TLArg(getXrPath(suggestedBindings->interactionProfile).c_str(), "InteractionProfile"));

            const std::string& interactionProfile = getXrPath(suggestedBindings->interactionProfile);
            if (!isPassthrough() && interactionProfile == "/interaction_profiles/valve/index_controller") {
                for (uint32_t i = 0; i < suggestedBindings->countSuggestedBindings; i++) {
                    TraceLoggingWrite(
                        g_traceProvider,
                        "xrSuggestInteractionProfileBindings",
                        TLXArg(suggestedBindings->suggestedBindings[i].action, "Action"),
                        TLArg(getXrPath(suggestedBindings->suggestedBindings[i].binding).c_str(), "Path"));

                    const std::string& path = getXrPath(suggestedBindings->suggestedBindings[i].binding);
                    if (path == "/user/hand/left/input/grip/pose" || path == "/user/hand/left/input/grip") {
                        m_gripAction[xr::Side::Left] = suggestedBindings->suggestedBindings[i].action;
                    } else if (path == "/user/hand/right/input/grip/pose" || path == "/user/hand/right/input/grip") {
                        m_gripAction[xr::Side::Right] = suggestedBindings->suggestedBindings[i].action;
                    }
                }
            }

            if (m_gripAction[xr::Side::Left] == XR_NULL_HANDLE) {
                // TODO: Add grip pose if not present.
            }
            if (m_gripAction[xr::Side::Right] == XR_NULL_HANDLE) {
                // TODO: Add grip pose if not present.
            }

            const XrResult result = OpenXrApi::xrSuggestInteractionProfileBindings(instance, suggestedBindings);

            return result;
        }

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrCreateHandTrackerEXT
        XrResult xrCreateHandTrackerEXT(XrSession session,
                                        const XrHandTrackerCreateInfoEXT* createInfo,
                                        XrHandTrackerEXT* handTracker) {
            if (createInfo->type != XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT) {
                return XR_ERROR_VALIDATION_FAILURE;
            }

            TraceLoggingWrite(g_traceProvider,
                              "xrCreateHandTrackerEXT",
                              TLXArg(session, "Session"),
                              TLArg((uint32_t)createInfo->hand, "Hand"),
                              TLArg((uint32_t)createInfo->handJointSet, "HandJointSet"));

            XrResult result = XR_ERROR_RUNTIME_FAILURE;
            if (isSessionHandled(session)) {
                if ((createInfo->hand != XR_HAND_LEFT_EXT && createInfo->hand != XR_HAND_RIGHT_EXT) ||
                    createInfo->handJointSet != XR_HAND_JOINT_SET_DEFAULT_EXT) {
                    return XR_ERROR_VALIDATION_FAILURE;
                }

                std::unique_lock lock(m_handTrackersMutex);

                HandTracker& xrHandTracker = *new HandTracker;
                xrHandTracker.side = createInfo->hand == XR_HAND_LEFT_EXT ? xr::Side::Left : xr::Side::Right;

                *handTracker = (XrHandTrackerEXT)&xrHandTracker;

                // Maintain a list of known trackers for validation.
                m_handTrackers.insert(*handTracker);

                result = XR_SUCCESS;
            } else {
                result = OpenXrApi::xrCreateHandTrackerEXT(session, createInfo, handTracker);
            }

            if (XR_SUCCEEDED(result)) {
                TraceLoggingWrite(g_traceProvider, "xrCreateHandTrackerEXT", TLXArg(*handTracker, "HandTracker"));
            }

            return result;
        }

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrDestroyHandTrackerEXT
        XrResult xrDestroyHandTrackerEXT(XrHandTrackerEXT handTracker) {
            TraceLoggingWrite(g_traceProvider, "xrDestroyHandTrackerEXT", TLXArg(handTracker, "HandTracker"));

            std::unique_lock lock(m_handTrackersMutex);

            if (m_handTrackers.count(handTracker)) {
                HandTracker* xrHandTracker = (HandTracker*)handTracker;

                delete xrHandTracker;
                m_handTrackers.erase(handTracker);
                return XR_SUCCESS;
            }

            return OpenXrApi::xrDestroyHandTrackerEXT(handTracker);
        }

        // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#xrLocateHandJointsEXT
        XrResult xrLocateHandJointsEXT(XrHandTrackerEXT handTracker,
                                       const XrHandJointsLocateInfoEXT* locateInfo,
                                       XrHandJointLocationsEXT* locations) {
            if (locateInfo->type != XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT ||
                locations->type != XR_TYPE_HAND_JOINT_LOCATIONS_EXT) {
                return XR_ERROR_VALIDATION_FAILURE;
            }

            TraceLoggingWrite(g_traceProvider,
                              "xrLocateHandJointsEXT",
                              TLXArg(handTracker, "HandTracker"),
                              TLArg(locateInfo->time),
                              TLXArg(locateInfo->baseSpace));

            std::unique_lock lock(m_handTrackersMutex);

            if (m_handTrackers.count(handTracker)) {
                XrHandJointVelocitiesEXT* velocities = reinterpret_cast<XrHandJointVelocitiesEXT*>(locations->next);
                while (velocities) {
                    if (velocities->type == XR_TYPE_HAND_JOINT_VELOCITIES_EXT) {
                        break;
                    }
                    velocities = reinterpret_cast<XrHandJointVelocitiesEXT*>(velocities->next);
                }

                if (locations->jointCount != XR_HAND_JOINT_COUNT_EXT ||
                    (velocities && velocities->jointCount != XR_HAND_JOINT_COUNT_EXT)) {
                    return XR_ERROR_VALIDATION_FAILURE;
                }

                HandTracker& xrHandTracker = *(HandTracker*)handTracker;

                locations->isActive = XR_FALSE;
                do {
                    // Check whether the controller is currently reported as an Index controller.
                    XrInteractionProfileState interactionProfile{XR_TYPE_INTERACTION_PROFILE_STATE};
                    CHECK_XRCMD(OpenXrApi::xrGetCurrentInteractionProfile(
                        m_session, m_subActionPath[xrHandTracker.side], &interactionProfile));
                    TraceLoggingWrite(
                        g_traceProvider,
                        "xrLocateHandJointsEXT_GetCurrentInteractionProfile",
                        TLArg(getXrPath(interactionProfile.interactionProfile).c_str(), "InteractionProfile"));
                    if (getXrPath(interactionProfile.interactionProfile) !=
                        "/interaction_profiles/valve/index_controller") {
                        break;
                    }

                    ensureResources();

                    XrSpaceLocation baseToGrip{XR_TYPE_SPACE_LOCATION};
                    const XrResult result = OpenXrApi::xrLocateSpace(
                        m_gripSpace[xrHandTracker.side], locateInfo->baseSpace, locateInfo->time, &baseToGrip);
                    if (XR_FAILED(result)) {
                        TraceLoggingWrite(g_traceProvider,
                                          "xrLocateHandJointsEXT_LocateSpaceError",
                                          TLArg(xr::ToCString(result), "Result"));
                        return result;
                    }
                    TraceLoggingWrite(g_traceProvider,
                                      "xrLocateHandJointsEXT_LocateSpace",
                                      TLArg(baseToGrip.locationFlags, "LocationFlags"));

                    EVRInputError error;

                    // TODO: This is likely redudant/interfere with VarjoOpenXR.
                    VRActiveActionSet_t actionSet{};
                    actionSet.ulActionSet = m_actionSet;
                    error = VRInput()->UpdateActionState(&actionSet, sizeof(VRActiveActionSet_t), 1);
                    TraceLoggingWrite(
                        g_traceProvider, "xrLocateHandJointsEXT_UpdateActionState", TLArg((int)error, "Error"));

                    InputSkeletalActionData_t actionData{};
                    error = VRInput()->GetSkeletalActionData(
                        m_skeletonAction[xrHandTracker.side], &actionData, sizeof(actionData));
                    TraceLoggingWrite(g_traceProvider,
                                      "xrLocateHandJointsEXT_GetSkeletalActionData",
                                      TLArg((int)error, "Error"),
                                      TLArg(!!actionData.bActive, "Active"));

                    uint32_t boneCount = 0;
                    VRInput()->GetBoneCount(m_skeletonAction[xrHandTracker.side], &boneCount);

                    VRBoneTransform_t bones[HandSkeletonBone::eBone_Count];
                    error = VRInput()->GetSkeletalBoneData(m_skeletonAction[xrHandTracker.side],
                                                           VRSkeletalTransformSpace_Parent,
                                                           VRSkeletalMotionRange_WithController,
                                                           bones,
                                                           (uint32_t)std::size(bones));
                    TraceLoggingWrite(
                        g_traceProvider, "xrLocateHandJointsEXT_GetSkeletalBoneData", TLArg((int)error, "Error"));
                    locations->isActive = error == VRInputError_None;

                    if (locations->isActive != XR_TRUE && !Pose::IsPoseValid(baseToGrip.locationFlags)) {
                        break;
                    }

                    // We must apply the transforms in order of the bone structure:
                    // https://github.com/ValveSoftware/openvr/wiki/Hand-Skeleton#bone-structure
                    XrVector3f barycenter{};
                    XrPosef accumulatedPose = baseToGrip.pose;
                    XrPosef wristPose;
                    for (uint32_t i = 0; i < locations->jointCount; i++) {
                        accumulatedPose = Pose::Multiply(
                            Pose::MakePose(
                                XrQuaternionf{bones[i].orientation.x,
                                              bones[i].orientation.y,
                                              bones[i].orientation.z,
                                              bones[i].orientation.w},
                                XrVector3f{bones[i].position.v[0], bones[i].position.v[1], bones[i].position.v[2]}),
                            accumulatedPose);

                        // Palm is estimated after this loop.
                        if (i != XR_HAND_JOINT_PALM_EXT) {
                            locations->jointLocations[i].radius = 0.005f;

                            // We need extra rotations to convert from what SteamVR expects to what OpenXR expects.
                            XrPosef correctedPose;
                            if (i != XR_HAND_JOINT_WRIST_EXT) {
                                correctedPose =
                                    Pose::Multiply(Pose::MakePose(Quaternion::RotationRollPitchYaw(
                                                                      {DegreeToRad(!xrHandTracker.side ? 0.f : 180.f),
                                                                       DegreeToRad(-90.f),
                                                                       DegreeToRad(180.f)}),
                                                                  XrVector3f{0, 0, 0}),
                                                   accumulatedPose);
                            } else {
                                correctedPose = Pose::Multiply(
                                    Pose::MakePose(Quaternion::RotationRollPitchYaw(
                                                       {DegreeToRad(180.f),
                                                        DegreeToRad(0.f),
                                                        DegreeToRad(!xrHandTracker.side ? -90.f : 90.f)}),
                                                   XrVector3f{0, 0, 0}),
                                    accumulatedPose);
                            }
                            locations->jointLocations[i].pose = correctedPose;
                        }
                        locations->jointLocations[i].locationFlags =
                            (XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT) |
                            baseToGrip.locationFlags;

                        switch (i) {
                        case XR_HAND_JOINT_WRIST_EXT:
                            wristPose = accumulatedPose;
                            break;

                        case XR_HAND_JOINT_INDEX_METACARPAL_EXT:
                        case XR_HAND_JOINT_INDEX_PROXIMAL_EXT:
                        case XR_HAND_JOINT_MIDDLE_METACARPAL_EXT:
                        case XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT:
                        case XR_HAND_JOINT_RING_METACARPAL_EXT:
                        case XR_HAND_JOINT_RING_PROXIMAL_EXT:
                        case XR_HAND_JOINT_LITTLE_METACARPAL_EXT:
                        case XR_HAND_JOINT_LITTLE_PROXIMAL_EXT:
                            barycenter = barycenter + accumulatedPose.position;
                            break;

                        // Reset to the wrist base pose once we reach the tip.
                        case XR_HAND_JOINT_THUMB_TIP_EXT:
                        case XR_HAND_JOINT_INDEX_TIP_EXT:
                        case XR_HAND_JOINT_MIDDLE_TIP_EXT:
                        case XR_HAND_JOINT_RING_TIP_EXT:
                        case XR_HAND_JOINT_LITTLE_TIP_EXT:
                            accumulatedPose = wristPose;
                            break;
                        }

                        if (velocities) {
                            velocities->jointVelocities[i].angularVelocity = {};
                            velocities->jointVelocities[i].linearVelocity = {};
                            velocities->jointVelocities[i].velocityFlags = 0;
                        }
                    }

                    // SteamVR doesn't have palm, we compute the barycenter of the metacarpal and proximal for
                    // index/middle/ring/little fingers.
                    barycenter = barycenter / 8.0f;
                    locations->jointLocations[XR_HAND_JOINT_PALM_EXT].radius = 0.04f;
                    locations->jointLocations[XR_HAND_JOINT_PALM_EXT].pose = Pose::MakePose(
                        locations->jointLocations[XR_HAND_JOINT_MIDDLE_METACARPAL_EXT].pose.orientation, barycenter);
                } while (false);

                if (locations->isActive != XR_TRUE) {
                    // If base space pose is not valid, we cannot locate.
                    TraceLoggingWrite(g_traceProvider, "xrLocateHandJointsEXT", TLArg(0, "LocationFlags"));
                    for (uint32_t i = 0; i < locations->jointCount; i++) {
                        locations->jointLocations[i].radius = 0.0f;
                        locations->jointLocations[i].pose = Pose::Identity();
                        locations->jointLocations[i].locationFlags = 0;

                        if (velocities) {
                            velocities->jointVelocities[i].angularVelocity = {};
                            velocities->jointVelocities[i].linearVelocity = {};
                            velocities->jointVelocities[i].velocityFlags = 0;
                        }
                    }
                }

                return XR_SUCCESS;
            }

            return OpenXrApi::xrLocateHandJointsEXT(handTracker, locateInfo, locations);
        }

      private:
        struct HandTracker {
            int side;
        };

        void ensureResources() {
            if (!m_manifestUpdated) {
                const EVRInputError error = original_SetActionManifestPath(
                    VRInput(), (dllHome / "VarjoOpenXRActions" / "OpenXrActionManifest.json").string().c_str());
                Log(fmt::format("SetActionManifestPath returned: {}\n", error).c_str());
                m_manifestUpdated = true;
            }

            if (m_actionSet == k_ulInvalidActionSetHandle) {
                const EVRInputError error =
                    VRInput()->GetActionSetHandle("/actions/controller_actions_scene_actionset", &m_actionSet);
                Log(fmt::format("GetActionSetHandle returned: {}\n", error).c_str());
            }

            for (uint32_t i = 0; i < xr::Side::Count; i++) {
                if (m_gripSpace[i] == XR_NULL_HANDLE) {
                    XrActionSpaceCreateInfo actionSpaceInfo{XR_TYPE_ACTION_SPACE_CREATE_INFO};
                    actionSpaceInfo.action = m_gripAction[i];
                    actionSpaceInfo.subactionPath = m_subActionPath[i];
                    actionSpaceInfo.poseInActionSpace = Pose::Identity();

                    CHECK_XRCMD(xrCreateActionSpace(m_session, &actionSpaceInfo, &m_gripSpace[i]));
                }

                if (m_skeletonAction[i] == k_ulInvalidActionHandle) {
                    const EVRInputError error = VRInput()->GetActionHandle(
                        i == xr::Side::Left ? "/actions/controller_actions_scene_actionset/in/HandSkeletonLeft"
                                            : "/actions/controller_actions_scene_actionset/in/HandSkeletonRight",
                        &m_skeletonAction[i]);
                    Log(fmt::format("GetActionHandle returned: {}\n", error).c_str());
                }
            }
        }

        const std::string getXrPath(XrPath path) {
            if (path == XR_NULL_PATH) {
                return "";
            }

            char buf[XR_MAX_PATH_LENGTH];
            uint32_t count;
            CHECK_XRCMD(OpenXrApi::xrPathToString(GetXrInstance(), path, sizeof(buf), &count, buf));
            std::string str;
            str.assign(buf, count - 1);
            return str;
        }

        bool isSystemHandled(XrSystemId systemId) const {
            return systemId == m_systemId;
        }

        bool isSessionHandled(XrSession session) const {
            return session == m_session;
        }

        bool isPassthrough() const {
            return false;
        }

        static inline EVRInputError (*original_SetActionManifestPath)(IVRInput* input,
                                                                      const char* pchActionManifestPath) = nullptr;
        static EVRInputError hooked_SetActionManifestPath(IVRInput* input, const char* pchActionManifestPath) {
            const EVRInputError error = original_SetActionManifestPath(
                input, (dllHome / "VarjoOpenXRActions" / "OpenXrActionManifest.json").string().c_str());
            if (error == VRInputError_None) {
                //                m_manifestUpdated = true;
            }
            return error;
        }

        bool m_bypassApiLayer{false};
        XrSystemId m_systemId{XR_NULL_SYSTEM_ID};
        XrSession m_session{XR_NULL_HANDLE};
        XrPath m_subActionPath[xr::Side::Count]{XR_NULL_PATH, XR_NULL_PATH};
        XrAction m_gripAction[xr::Side::Count]{XR_NULL_HANDLE, XR_NULL_HANDLE};
        XrSpace m_gripSpace[xr::Side::Count]{XR_NULL_HANDLE, XR_NULL_HANDLE};

        bool m_manifestUpdated{false};
        VRActionSetHandle_t m_actionSet{k_ulInvalidActionSetHandle};
        VRActionHandle_t m_skeletonAction[xr::Side::Count]{k_ulInvalidActionHandle, k_ulInvalidActionHandle};

        std::mutex m_handTrackersMutex;
        std::set<XrHandTrackerEXT> m_handTrackers;
    };

    // This method is required by the framework to instantiate your OpenXrApi implementation.
    OpenXrApi* GetInstance() {
        if (!g_instance) {
            g_instance = std::make_unique<OpenXrLayer>();
        }
        return g_instance.get();
    }

} // namespace openxr_api_layer

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    switch (ul_reason_for_call) {
    case DLL_PROCESS_ATTACH:
        TraceLoggingRegister(openxr_api_layer::log::g_traceProvider);
        break;

    case DLL_PROCESS_DETACH:
        TraceLoggingUnregister(openxr_api_layer::log::g_traceProvider);
        break;

    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
        break;
    }
    return TRUE;
}
