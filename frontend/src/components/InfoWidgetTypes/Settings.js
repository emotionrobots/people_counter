import React from 'react'

function Settings(props) {
    return (
        <div className='flex flex-col p-2 overflow-auto h-full'>
            {
                props.data.map((val, ind) => (
                    <div key={ind} className='flex'>
                        <div className='text-white font-semibold p-1 break-words'>{val.name + ': ' + val.current}</div>
                    </div>
                ))
            }
        </div>
    )
}

export default Settings
