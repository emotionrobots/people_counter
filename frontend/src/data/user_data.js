import { InfoWidgetTypes } from "../components/InfoWidget";
import Auth from "@aws-amplify/auth";

let USER_CONTEXT = null
let USER_TOKEN = null
let USER_ATTRIBUTES = null
const LINK = "http://localhost:3000"
const LINK_MAND_ADDON = ""

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
        console.log(USER_TOKEN)
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
                        USER_CONTEXT = {
                            username: USER_ATTRIBUTES.username,
                            token: '',
                            error: 0,
                            organizations: data
                            // organizations: {
                            //     'organizationID0': {
                            //         name: 'Yummy',
                            //         //desc: 'Delicious area located in the blah blah what if something is too long ehe te nandeyo wowowowowowowooww omg why is this still not long enogh to overfvlow please hurry up whyyyyyyy fjewioafj eiwfowjf weoifj eowi furry up whyyyyyyy fjewioafj eiwfowjf weoifj eowi furry up whyyyyyyy fjewioafj eiwfowjf weoifj eowi furry up whyyyyyyy fjewioafj eiwfowjf weoifj eowi fjoiwaj oiejfj awoij',
                            //         desc: 'jifoawiofj',
                            //         date_creation: null,
                            //         cameraGroups: {
                            //             'groupID0': {
                            //                 name: 'CA',
                            //                 cameras: [
                            //                     {
                            //                         name: 'cam 1'
                            //                     },
                            //                     {
                            //                         name: 'backup cam 1',
                            //                         desc: 'the best backup camera in the world',
                            //                         date_creation: '12/01/2009 18:21'
                            //                     },
                            //                     {
                            //                         name: 'trooper 1',
                            //                         desc: 'an absolute beast of a camera',
                            //                         date_creation: '06/20/2004 06:21'
                            //                     }
                            //                 ]
                            //             },
                            //             'groupID1': {
                            //                 name: 'CA1',
                            //                 cameras: [
                            //                     {
                            //                         name: 'cam 2'
                            //                     }
                            //                 ]
                            //             },
                            //             'groupID2': {
                            //                 name: 'CA2',
                            //                 cameras: [
                            //                     {
                            //                         name: 'cam 3'
                            //                     }
                            //                 ]
                            //             },
                            //             'groupID3': {
                            //                 name: 'CA3',
                            //                 cameras: [
                            //                     {
                            //                         name: 'cam 4'
                            //                     }
                            //                 ]
                            //             },
                            //         }
                            //     },
                            //     'organizationID1': {
                            //         name: 'Clements',
                            //         date_creation: null,
                            //         cameraGroups: {
                            //             'groupID0': {
                            //                 name: 'Classroom A',
                            //                 cameras: [
                            //                     {
                            //                         name: 'cam 5'
                            //                     }
                            //                 ]
                            //             },
                            //             'groupID1': {
                            //                 name: 'Classroom B',
                            //                 cameras: [
                            //                     {
                            //                         name: 'cam 6'
                            //                     }
                            //                 ]
                            //             },
                            //         }
                            //     }
                            // }
                        };
                        callback(USER_CONTEXT)
                    })
                })
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

export function getInfoWidget(data, callback, currentSelectedCamera = [0, 0]) {
    switch (data) {
        case 'numOccupancy':
            __internal_fetch('/get_occupancy',
                POST_REQ,
                {},
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
                {},
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
                {},
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