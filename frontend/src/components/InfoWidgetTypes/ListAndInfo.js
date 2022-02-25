import React, { useContext, useState } from 'react'
import { StateContext } from '../Contexts/StateContext'
import { UserContext } from '../Contexts/UserContext'

import Settings from './Settings'

/*
let camarr = []
            
            // callback({
            //     cardType: InfoWidgetTypes.LISTPLUSINFO,
            //     attributes: {
            //         data: camarr
            //     }
            // })
            */

function getSettingsArr(data, currentSelected, userContext) {
    let USER_CONTEXT_ORGS = Object.values(userContext.organizations)[currentSelected[0]];
    let USER_CONTEXT_CAMGROUPS = Object.values(USER_CONTEXT_ORGS.cameraGroups)[currentSelected[1]];
    let USER_CONTEXT_CAM = USER_CONTEXT_CAMGROUPS.cameras[currentSelected[2]];

    switch (data) {
        case 0:
            return [
                {
                    name: 'Name',
                    current: USER_CONTEXT_ORGS.name,
                    settingType: 2
                },
                {
                    name: 'Date Created',
                    current: USER_CONTEXT_ORGS.date_creation,
                    settingType: 2
                },
                {
                    name: '# of Rooms',
                    current: Object.keys(USER_CONTEXT_ORGS.cameraGroups).length,
                    settingType: 2
                },
                {
                    name: 'Description',
                    current: USER_CONTEXT_ORGS.desc,
                    settingType: 2
                },
            ]
        case 1:
            return [
                {
                    name: 'Name',
                    current: USER_CONTEXT_CAMGROUPS.name,
                    settingType: 2
                },
                {
                    name: 'Date Created',
                    current: USER_CONTEXT_CAMGROUPS.date_creation,
                    settingType: 2
                },
                {
                    name: '# of Cameras',
                    current: (USER_CONTEXT_CAMGROUPS).cameras.length,
                    settingType: 2
                },
                {
                    name: 'Description',
                    current: USER_CONTEXT_CAMGROUPS.desc,
                    settingType: 2
                },
                {
                    name: 'Owned By',
                    current: USER_CONTEXT_ORGS.name,
                    settingType: 2
                }
            ]
        case 2:
            return [
                {
                    name: 'Name',
                    current: USER_CONTEXT_CAM.name,
                    settingType: 2
                },
                {
                    name: 'Date Created',
                    current: USER_CONTEXT_CAM.date_creation,
                    settingType: 2
                },
                {
                    name: 'Description',
                    current: USER_CONTEXT_CAM.desc,
                    settingType: 2
                },
                {
                    name: 'Room',
                    current: USER_CONTEXT_CAMGROUPS.name,
                    settingType: 2
                },
                {
                    name: 'Owned By',
                    current: USER_CONTEXT_ORGS.name,
                    settingType: 2
                }
            ]
        default:
            break;
    }
}

function getRelevantElementArray(data, currentSelected, userContext) {
    let arr = [];
    let context = [];

    if (data >= 0)
        context = Object.values(userContext.organizations)

    if (data >= 1)
        context = Object.values(context[currentSelected[0]].cameraGroups)

    if (data >= 2)
        context = context[currentSelected[1]].cameras

    for (let i = 0; i < context.length; ++i) {
        arr.push(context[i].name)
    }

    return arr;
}

function ListAndInfo(props) {
    const [state, dispatch] = useContext(StateContext)
    const [camSelected, setCamSelected] = useState(0)
    const usrContext = useContext(UserContext)

    let updatedSelectedCam = camSelected;

    if(props.data >= 2){
        //TODO: GET RID OF DUPLICATE
        let USER_CONTEXT_ORGS = Object.values(usrContext.organizations)[state.currentSelectedCamGroup[0]];
        let USER_CONTEXT_CAMGROUPS = Object.values(USER_CONTEXT_ORGS.cameraGroups)[state.currentSelectedCamGroup[1]];
        if(camSelected > (Object.values(USER_CONTEXT_CAMGROUPS.cameras).length - 1)){
            setCamSelected(0)
            updatedSelectedCam = 0;
        }
    }

    return (
        <div className={'h-full ' + (props.horz ? 'flex flex-row w-full' : '')}>
            <div className={'p-3 overflow-auto ' + (props.horz ? 'w-1/2' : 'h-1/3 ')}>
                {
                    getRelevantElementArray(props.data, state.currentSelectedCamGroup, usrContext).map((val, ind) => (
                        <div 
                        key={ind}
                        className={'flex p-3 m-1 rounded-xl text-white font-bold outline-none select-none ' + (ind === (props.data < 2 ? state.currentSelectedCamGroup[props.data % 2] : updatedSelectedCam) ? 'bg-blue-400' : 'bg-blue-500 cursor-pointer hover:border-gray-700 border-transparent border-2 hover:border-current')}
                        onClick={(e) => {
                            e.preventDefault()
                            if(props.data < 2){
                                let newCamGroupConfig = [...state.currentSelectedCamGroup]
                                newCamGroupConfig[props.data] = ind

                                dispatch({
                                    type: 'switch_cam_group', camGroup: newCamGroupConfig
                                })
                            }else{
                                setCamSelected(ind)
                            }
                        }}>{val}</div>
                    ))
                }
            </div>
            {!isNaN(props.data) ? <Settings data={getSettingsArr(props.data, [...state.currentSelectedCamGroup, updatedSelectedCam], usrContext)} className={"border-t-2 border-white border-opacity-100 " + props.horz ? 'w-1/2' : 'h-1/3'}></Settings> :
                <div>An Error Occured</div>}
        </div>
    )
}

export default ListAndInfo
