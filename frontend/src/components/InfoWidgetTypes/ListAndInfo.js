import React, { useContext, useState } from 'react'
import { StateContext } from '../Contexts/StateContext'
import { UserContext } from '../Contexts/UserContext'
import Dropdown from '../Dropdown'

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
            if(USER_CONTEXT_CAM != null){
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
            } else {
                return [
                    {
                        name: 'No Camera Exists',
                        current: '',
                        settingType: 2
                    }
                ]
            }
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

function getCardTitle(data){
    switch (data) {
        case 0:
            return 'ORGANIZATIONS'
        case 1:
            return 'ROOMS'
        case 2:
            return 'CAMERAS'
        default:
            break;
    }
}

function ListAndInfo(props) {
    const [state, dispatch] = useContext(StateContext)
    const [camSelected, setCamSelected] = useState(0)
    const usrContext = useContext(UserContext)

    let updatedSelectedCam = camSelected;

    if(Object.keys(usrContext.organizations).length == 0){
        return <div className="flex h-full w-full p-5 justify-center items-center text-center text-white font-bold text-lg">
            No Cameras Linked to Account
        </div>
    }

    if(props.data >= 2){
        //TODO: GET RID OF DUPLICATE
        let USER_CONTEXT_ORGS = Object.values(usrContext.organizations)[state.currentSelectedCamGroup[0]];
        let USER_CONTEXT_CAMGROUPS = Object.values(USER_CONTEXT_ORGS.cameraGroups)[state.currentSelectedCamGroup[1]];
        if(Object.values(USER_CONTEXT_CAMGROUPS.cameras).length > 0 && camSelected > (Object.values(USER_CONTEXT_CAMGROUPS.cameras).length - 1)){
            setCamSelected(0)
            updatedSelectedCam = 0;
        }
    }

    let elemArray = getRelevantElementArray(props.data, state.currentSelectedCamGroup, usrContext)

    //style={props.horz ? {'writingMode': 'vertical-rl', 'textOrientation': 'upright'} : {}}
    return (
        <div className={'h-full ' + (props.horz ? 'flex flex-row w-full' : '')}>
        <div className={'p-3 m-0 text-white font-bold writing-mode-vertical text-xl' + (props.horz ? ' w-0 break-words leading-2 tracking-tighter' : '')}>{getCardTitle(props.data)}</div>
            <div className={'p-3 overflow-auto ' + (props.horz ? 'w-1/2' : '')}>
                {
                    <Dropdown options={elemArray} selected={state.currentSelectedCamGroup[props.data]} onChange={
                        (e) => {
                            e.preventDefault()
                            if(props.data < 2){
                                let newCamGroupConfig = [...state.currentSelectedCamGroup]
                                newCamGroupConfig[props.data] = elemArray.indexOf(e.target.value)

                                dispatch({
                                    type: 'switch_cam_group', camGroup: newCamGroupConfig
                                })
                            }else{
                                setCamSelected(elemArray.indexOf(e.target.value))
                            }
                        }
                    }></Dropdown>
                    // getRelevantElementArray(props.data, state.currentSelectedCamGroup, usrContext).map((val, ind) => (
                    //     <div 
                    //     key={ind}
                    //     className={'flex p-3 m-1 rounded-xl text-white font-bold outline-none select-none ' + (ind === (props.data < 2 ? state.currentSelectedCamGroup[props.data % 2] : updatedSelectedCam) ? 'bg-blue-400' : 'bg-blue-500 cursor-pointer hover:border-gray-700 border-transparent border-2 hover:border-current')}
                    //     onClick={(e) => {
                    //         e.preventDefault()
                    //         if(props.data < 2){
                    //             let newCamGroupConfig = [...state.currentSelectedCamGroup]
                    //             newCamGroupConfig[props.data] = ind

                    //             dispatch({
                    //                 type: 'switch_cam_group', camGroup: newCamGroupConfig
                    //             })
                    //         }else{
                    //             setCamSelected(ind)
                    //         }
                    //     }}>{val}</div>
                    // ))
                }
            </div>
            {!isNaN(props.data) ? <Settings data={getSettingsArr(props.data, [...state.currentSelectedCamGroup, updatedSelectedCam], usrContext)} className={"border-t-2 border-white border-opacity-100 " + props.horz ? 'w-1/2' : 'h-1/3'}></Settings> :
                <div>An Error Occured</div>}
        </div>
    )
}

export default ListAndInfo
