import React, { useContext } from 'react';
import NavItem from './NavItem'
import { ViewGridIcon, UserIcon, CameraIcon, LogoutIcon } from '@heroicons/react/solid'
import {
    Link
} from 'react-router-dom';
import Dropdown from '../Dropdown'
import { getNamesOfCamGroups, getNamesOfOrgs, UserContext } from '../Contexts/UserContext';
import { StateContext } from '../Contexts/StateContext';
import Auth from '@aws-amplify/auth';
import CameraGroup from './CameraGroup';

function Sidebar() {
    const [state, dispatch] = useContext(StateContext)
    let userContext = useContext(UserContext)

    let orgNames = getNamesOfOrgs(userContext.organizations)

    return (
        <div className="relative h-full w-64 p-6 mr-4 rounded-3xl bg-gradient-to-b from-pink-500 to-purple-700">
            <div className="flex flex-row m-1 mb-6 w-60">
                <img className="rounded-full bg-white h-16 w-16 p-4 " src="assets/default_avatar.png" alt="avatar can't be loaded"></img>
                <p className="flex items-center px-3 right-0 font-bold text-white truncate">{userContext.username}</p>
            </div>
            <Link to={'/'}><NavItem icon={ViewGridIcon} desc="Dashboard"></NavItem></Link>
            <Link to={'/profile'}><NavItem icon={UserIcon} desc="Profile"></NavItem></Link>
            <Link to={'/cameragroups'}><NavItem icon={CameraIcon} desc="Camera Groups"></NavItem></Link>
            <hr className="mt-6 mb-4 border-0 text-white bg-white h-1"></hr>
            <Dropdown options={orgNames} onChange={
                (e) => {
                    e.preventDefault()
                    dispatch({
                        type: 'switch_cam_group', camGroup: [orgNames.indexOf(e.target.value), 0]
                    })
                }
            }></Dropdown>
            {
                // Map through the selected organization
                getNamesOfCamGroups(Object.values(userContext.organizations)[state.currentSelectedCamGroup[0]]).map((val, ind) => {
                    return <CameraGroup key={ind} id={ind} selected={ind === state.currentSelectedCamGroup[1]} camGroup={val} onClick={(e) => {
                        e.preventDefault()
                        dispatch({ type: 'switch_cam_group', camGroup: [state.currentSelectedCamGroup[0], ind] })
                    }} />
                })
            }
            <button
                className="absolute bottom-4"
                onClick={async (e) => {
                    dispatch({ type: 'start_loading', loadingMessage: 'Logging you out...' })
                    e.preventDefault()
                    await Auth.signOut()
                    window.location.reload()
                }}>
                <NavItem icon={LogoutIcon} desc="Logout"></NavItem>
            </button>
        </div>
    )
}

export default Sidebar;