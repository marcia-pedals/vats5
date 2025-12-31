import { useState, useEffect } from 'react'
import './App.css'

function App() {
  const [message, setMessage] = useState('Loading...')

  useEffect(() => {
    fetch('/api/')
      .then(res => res.text())
      .then(setMessage)
      .catch(err => setMessage('Error: ' + err.message))
  }, [])

  return (
    <div>
      <h1>VATS5</h1>
      <p>Backend says: {message}</p>
    </div>
  )
}

export default App
