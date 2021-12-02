import React from 'react';

export const UserContext = React.createContext(
    {
        username: 'Tester',
        token: '',
        organizations: {
            'organizationID0' : {
                name: 'Yummy',
                cameraGroups: {
                    'groupID0' : 'CA',
                    'groupID1' : 'CA1',
                    'groupID2' : 'CA2',
                    'groupID3' : 'CA3',
                }
            },
            'organizationID1' : {
                name: 'Clements',
                cameraGroups: {
                    'groupID0' : 'Classroom A',
                    'groupID1' : 'Classroom B',
                }
            }
        }
    }
)

/**
 * The organizations context is the organizations attributes under the global state. 
 * By searching through the the values of the organizations state value, the function pushes the names of the organizations onto a new array and returns it.
 * @param {OrganizationsContext} orgsContext 
 * @returns The names of all the registered organizations found in the OrganizationsContext given
 */
export function getNamesOfOrgs(orgsContext){
    let orgNames = []
    Object.values(orgsContext).forEach((val) => {
        orgNames.push(val.name)
    })

    return orgNames
}

/**
 * The organization context is an organization value under a oragnization ID# in the global state.
 * @param {OrganizationContext} orgContext 
 * @returns Names of all the camera groups that are available in the orgContext. (Value of the cameraGroups attribute)
 */
export function getNamesOfCamGroups(orgContext){
    return Object.values(orgContext.cameraGroups)
}