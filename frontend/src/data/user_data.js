import { InfoWidgetTypes } from "../components/InfoWidget";
import Auth from "@aws-amplify/auth";

let USER_CONTEXT = null
let USER_TOKEN = null
let USER_ATTRIBUTES = null
const LINK = "https://ec2-52-14-188-10.us-east-2.compute.amazonaws.com/"
const LINK_MAND_ADDON = "api"

const POST_REQ = {
    method: 'POST',
    headers: { 
        'Accept': 'application/json',
        'Content-Type': 'application/json'
     },
};

function objToQueryString(obj) {
    const keyValuePairs = [];
    for (const key in obj) {
        keyValuePairs.push(encodeURIComponent(key) + '=' + encodeURIComponent(obj[key]));
    }
    return keyValuePairs.join('&');
}

function __internal_fetch(link_addon, headers, body, handleError, isJSON, callback) {
    console.log(new URL(LINK_MAND_ADDON + link_addon + (Object.keys(body).length > 0 ? ('?' + objToQueryString(body)) : ''), LINK))
    retrieveUserToken((e) => {
        handleError(e)
    }, () => {
        console.log(body)
        body['token'] = USER_TOKEN
        fetch(new URL(LINK_MAND_ADDON + link_addon, LINK), {
            headers: headers.headers,
            method: headers.method,
            body: JSON.stringify(body),
        })
            .then(res => isJSON ? res.json() : res.text())
            .then(data => callback(data))
            .catch(reason => handleError(reason))
    })
}

// Subscribe to MQTT 

function retrieveUserToken(errHandler, callback) {
    Auth.currentSession()
        .then((session) => {
            USER_TOKEN = session.accessToken.jwtToken
            USER_ATTRIBUTES = {
                username: session.idToken.payload['cognito:username'],
                email: session.idToken.payload.email,
                phone_number: session.idToken.payload.phone_number,
                email_verified: session.idToken.payload.email_verified,
                phone_number_verified: session.idToken.payload.phone_number_verified,
            }

            callback()
        })
        .catch(errHandler)
}

export function getUserContext(callback) {
    // User profile will write to S3
    // User profile image will be a link referencing the image

    retrieveUserToken((e) => { console.log(e) }, () => {
    __internal_fetch('/get_user_orgs',
                POST_REQ,
                {},
                reason => {
                    console.log(reason);
                    USER_CONTEXT = {
                        username: USER_ATTRIBUTES.username,
                        token: '',
                        error: 1,
                        organizations: {}
                    }
                    callback(USER_CONTEXT)
                },
                true,
                (data) => {
                    console.log(data)
                        USER_CONTEXT = {
                            username: USER_ATTRIBUTES.username,
                            token: '',
                            error: 0,
                            organizations: data
                        };
                        callback(USER_CONTEXT)
                    })
                })
}

export function get_history(start, end, callback, currentSelectedCamera){
    __internal_fetch('/get_history',
    POST_REQ,
    {
        start: start,
        end: end,
        room_id: Object.keys(Object.values(USER_CONTEXT.organizations)[currentSelectedCamera[0]].cameraGroups)[currentSelectedCamera[1]]
    },
    reason => {
        console.log(reason);
        callback("Error")
    },
    true,
    (data) => {
        console.log(data)
        callback(data)
    })
}

export function getInfoWidget(data, callback, currentSelectedCamera = [0, 0]) {
    switch (data) {
        case 'numOccupancy':
            __internal_fetch('/get_occupancy',
                POST_REQ,
                {room_id: Object.keys(Object.values(USER_CONTEXT.organizations)[currentSelectedCamera[0]].cameraGroups)[currentSelectedCamera[1]]},
                reason => {
                    console.log(reason);
                    callback("Error")
                },
                true,
                (data) => {
                    console.log(data)
                    callback(data)
                })
            break;
        case 'graphReport':
            callback({
                cardType: InfoWidgetTypes.CHART,
                attributes: {
                    data: [],
                }
            })
            break;
        case 'numEntered':
            __internal_fetch('/get_hourly_poschange',
                POST_REQ,
                {room_id: Object.keys(Object.values(USER_CONTEXT.organizations)[currentSelectedCamera[0]].cameraGroups)[currentSelectedCamera[1]]},
                reason => {
                    console.log(reason);
                    callback("Error")
                },
                true,
                (data) => {
                    console.log(data)
                    callback(data)
                })
            break;
        case 'numLeft':
            __internal_fetch('/get_hourly_negchange',
                POST_REQ,
                {room_id: Object.keys(Object.values(USER_CONTEXT.organizations)[currentSelectedCamera[0]].cameraGroups)[currentSelectedCamera[1]]},
                reason => {
                    console.log(reason);
                    callback("Error")
                },
                true,
                (data) => {
                    console.log(data)
                    callback(data)
                })
            break;
        case 'myUserInfo':
            /**
             * Setting Types
             * 0: Editable
             * 1: Secure Resettable (password)
             * 2: Uneditable
             * 3: Boolean Uneditable (Check or X)
             */
            callback({
                cardType: InfoWidgetTypes.SETTINGS,
                attributes: {
                    data: [
                        {
                            name: 'Username',
                            current: USER_ATTRIBUTES.username,
                            settingType: 2
                        },
                        {
                            name: 'E-Mail',
                            current: USER_ATTRIBUTES.email,
                            settingType: 1
                        },
                        {
                            name: 'Phone Number',
                            current: USER_ATTRIBUTES.phone_number,
                            settingType: 1
                        },
                        {
                            name: 'E-Mail Verified?',
                            current: USER_ATTRIBUTES.email_verified,
                            settingType: 3
                        },
                        {
                            name: 'Phone Number Verified?',
                            current: USER_ATTRIBUTES.phone_number_verified,
                            settingType: 3
                        },
                    ]
                }
            })
            break;
        case 'orgs':
            callback({
                cardType: InfoWidgetTypes.LISTPLUSINFO,
                attributes: {
                    data: 0,
                    horz: false
                }
            })
            break;
        case 'camGroups':
            callback({
                cardType: InfoWidgetTypes.LISTPLUSINFO,
                attributes: {
                    data: 1,
                    horz: false
                }
            })
            break;
        case 'profileInfo':
            callback({
                cardType: InfoWidgetTypes.SINGLE,
                attributes: {
                    data: USER_ATTRIBUTES.username,
                    icon: "profile"
                }
            })
            break;
        case 'orgs-horz':
            callback({
                cardType: InfoWidgetTypes.LISTPLUSINFO,
                attributes: {
                    data: 2,
                    horz: true
                }
            })
            break;
        default:

            break;
    }
}