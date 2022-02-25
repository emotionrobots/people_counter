//import { RefreshIcon } from '@heroicons/react/solid';
import React from 'react'
import {VscPerson} from 'react-icons/vsc'
import { ImExit, ImEnter } from 'react-icons/im'

function findIcon(icon){
    var iconClass = 'h-24 w-24 text-9xl text-white font-bold';
    switch(icon){
        case "human":
            return <VscPerson className={iconClass}></VscPerson>
        case "exit":
            return <ImExit className={iconClass}></ImExit>
        case "enter":
            return <ImEnter className={iconClass}></ImEnter>
        default:
            return <VscPerson className={iconClass}></VscPerson>
    }
}

function Single(props) {
    return (
        // <div className='flex relative flex-col h-full'>
        //     <RefreshIcon className='flex absolute top-2 right-2 h-5 w-5 text-9xl text-white font-bold'></RefreshIcon>
            <div className='flex flex-row h-full justify-center items-center pl-6 pr-3'>
                <p className='text-white text-4xl font-bold'>{props.data.data}</p>
                {findIcon(props.data.icon)}
            </div>
        // </div>
    )
}

export default Single