const express = require('express')

const app = express()
const SERVER_PORT = 5000

app.use(express.static(__dirname + '/html'))

app.use((req, res) => {

    res.status(404).send('Not Found')

})

const HTTPServer = app.listen(SERVER_PORT, () => {

    console.info(`Server is running on ${SERVER_PORT} port`)


})