import { InfoWidgetTypes } from "../components/InfoWidget";
import Auth from "@aws-amplify/auth";

let USER_TOKEN = null
let USER_ATTRIBUTES = null
let link = "http://localhost:3000/api"

const GET_HEADERS = {
    method: 'GET',
    headers: { 'Content-Type': 'plain/text', 'X-Requested-With': 'XMLHttpRequest' },
};

function __internal_fetch(link_addon, headers, params, handleError, isJSON, callback) {
    retrieveUserToken((e) => {
        handleError(e)
    }, () => {
        fetch(URL(link_addon, link), headers)
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
        callback({
            username: USER_ATTRIBUTES.username,
            token: '',
            organizations: {
                'organizationID0': {
                    name: 'Yummy',
                    cameraGroups: {
                        'groupID0': 'CA',
                        'groupID1': 'CA1',
                        'groupID2': 'CA2',
                        'groupID3': 'CA3',
                    }
                },
                'organizationID1': {
                    name: 'Clements',
                    cameraGroups: {
                        'groupID0': 'Classroom A',
                        'groupID1': 'Classroom B',
                    }
                }
            }
        })
    })
}

/**
 * Retrieves the username of the current user as a plain text.
 * @param {function(string)} callback Returns the data to this function after fetched. 
 */
export function getUsername(callback) {
    //TODO: Communicate with MQTT servers
    __internal_fetch('',
        GET_HEADERS,
        {},
        reason => {
            console.log(reason);
            callback("Error")
        },
        false,
        callback)
}

/**
 * Retrieves the organizations under this account in an array format.
 * @param {function(string)} callback Returns the data to this function after fetched. 
 */
export function getOrganizations(callback) {
    return [
        "My Groups",
        "Yummy",
        "Clements",
        "New One"
    ]
}

export function getInfoWidget(data, callback) {
    switch (data) {
        case 'numOccupancy':
            __internal_fetch('/update_counter',
                GET_HEADERS,
                {},
                reason => {
                    console.log(reason);
                    callback("Error")
                },
                false,
                (data) => callback({
                    cardType: InfoWidgetTypes.SINGLE,
                    attributes: {
                        data: data + " People in the Room",
                        icon: "human"
                    }
                }))
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
            callback({
                cardType: InfoWidgetTypes.SINGLE,
                attributes: {
                    data: 21 + " People Entered in the Past Hour",
                    icon: "enter"
                }
            })
            break;
        case 'numLeft':
            callback({
                cardType: InfoWidgetTypes.SINGLE,
                attributes: {
                    data: 18 + " People Left in the Past Hour",
                    icon: "exit"
                }
            })
            break;
        default:
            break;
    }
}