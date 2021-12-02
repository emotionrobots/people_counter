import React from 'react'

function Report(props) {
    return (
        <div className='flex flex-row h-full justify-center items-center p-3 pl-6 pr-3'>
            <p className='text-white text-lg font-bold'>{props.data}</p>

        </div>
    )
}

export default Report
